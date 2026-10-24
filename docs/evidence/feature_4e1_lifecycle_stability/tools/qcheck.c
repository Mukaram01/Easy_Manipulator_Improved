// qcheck: LD_PRELOAD allocator shim. Freed blocks are poisoned and held in a
// quarantine; a block whose poison changed before it is released is a
// use-after-free write. Reports the block, the bytes written and the stack that
// freed it, then aborts so a core is kept. Minimal overhead: real thread timing.
#define _GNU_SOURCE
#include <dlfcn.h>
#include <execinfo.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <malloc.h>

#define QSLOTS 65536
#define QBYTES (96u << 20)
#define DEPTH 10
#define POISON 0xD5

typedef struct { void * p; size_t n; void * caller; unsigned long tid; } slot_t;
static slot_t q[QSLOTS];
static size_t qhead, qcount, qbytes, total_frees, total_verified;
static pthread_mutex_t qlock = PTHREAD_MUTEX_INITIALIZER;
static __thread int inside;
static void *(*real_malloc)(size_t);
static void (*real_free)(void *);
static void *(*real_calloc)(size_t, size_t);
static void *(*real_realloc)(void *, size_t);
static int (*real_posix_memalign)(void **, size_t, size_t);
static void *(*real_aligned_alloc)(size_t, size_t);
static void *(*real_memalign)(size_t, size_t);
static int ready;
static char bootstrap[1 << 20];
static size_t bootstrap_used;

static int initializing;

static void * bootstrap_alloc(size_t n)
{
  size_t a = (n + 15) & ~(size_t)15;
  if (bootstrap_used + a > sizeof bootstrap) return NULL;
  void * p = bootstrap + bootstrap_used; bootstrap_used += a; return p;
}

static void init(void)
{
  if (ready) return;
  initializing = 1;  // dlsym may allocate: those requests use the bootstrap buffer
  real_malloc = dlsym(RTLD_NEXT, "malloc");
  real_free = dlsym(RTLD_NEXT, "free");
  real_calloc = dlsym(RTLD_NEXT, "calloc");
  real_realloc = dlsym(RTLD_NEXT, "realloc");
  real_posix_memalign = dlsym(RTLD_NEXT, "posix_memalign");
  real_aligned_alloc = dlsym(RTLD_NEXT, "aligned_alloc");
  real_memalign = dlsym(RTLD_NEXT, "memalign");
  ready = 1;
  initializing = 0;
  void * warm[DEPTH]; inside = 1; backtrace(warm, DEPTH); inside = 0;  // loads libgcc now (used only when reporting)
}

static void report(slot_t * s, size_t off)
{
  char line[512];
  int n = snprintf(line, sizeof line, "\n[qcheck] USE-AFTER-FREE WRITE: block %p size %zu written at offset %zu after free; bytes:",
                   s->p, s->n, off);
  write(2, line, n);
  unsigned char * b = s->p;
  size_t start = off & ~(size_t)15, end = start + 48 < s->n ? start + 48 : s->n;
  for (size_t i = start; i < end; ++i) { n = snprintf(line, sizeof line, "%s%02x", (i - start) % 8 ? "" : " ", b[i]); write(2, line, n); }
  n = snprintf(line, sizeof line, "\n[qcheck] freed from call site %p on thread %lu\n", s->caller, s->tid); write(2, line, n);
  backtrace_symbols_fd(&s->caller, 1, 2);
  n = snprintf(line, sizeof line, "[qcheck] detected on thread %lu, backtrace:\n", (unsigned long)pthread_self()); write(2, line, n);
  void * here[24]; int k = backtrace(here, 24); backtrace_symbols_fd(here, k, 2);
  abort();
}

static void verify(slot_t * s)
{
  unsigned char * b = s->p;
  for (size_t i = 0; i < s->n; ++i) if (b[i] != POISON) report(s, i);
}

static void release_oldest(void)
{
  slot_t * s = &q[(qhead + QSLOTS - qcount) % QSLOTS];
  verify(s); ++total_verified;
  qbytes -= s->n; --qcount;
  real_free(s->p);
}

void free(void * p)
{
  if (!p) return;
  init();
  if ((char *)p >= bootstrap && (char *)p < bootstrap + sizeof bootstrap) return;
  if (inside) { real_free(p); return; }
  inside = 1;
  size_t n = malloc_usable_size(p);
  if (n > (8u << 20)) { real_free(p); inside = 0; return; }  // keep huge blocks out of the quarantine
  memset(p, POISON, n);
  slot_t s; s.p = p; s.n = n; s.caller = __builtin_return_address(0); s.tid = (unsigned long)pthread_self();
  pthread_mutex_lock(&qlock);
  while (qcount && (qcount >= QSLOTS || qbytes + n > QBYTES)) release_oldest();
  q[qhead] = s; qhead = (qhead + 1) % QSLOTS; ++qcount; qbytes += n; ++total_frees;
  pthread_mutex_unlock(&qlock);
  inside = 0;
}

void * malloc(size_t n)
{
  if (!ready) { if (initializing) return bootstrap_alloc(n); init(); }
  return real_malloc(n);
}

void * calloc(size_t m, size_t n)
{
  if (!ready) { if (initializing) { void * p = bootstrap_alloc(m * n); if (p) memset(p, 0, m * n); return p; } init(); }
  return real_calloc(m, n);
}

void * realloc(void * p, size_t n)
{
  init();
  if (!p) return real_malloc(n);
  if ((char *)p >= bootstrap && (char *)p < bootstrap + sizeof bootstrap) { void * r = real_malloc(n); memcpy(r, p, n); return r; }
  void * r = real_malloc(n);
  if (!r) return NULL;
  size_t old = malloc_usable_size(p);
  memcpy(r, p, old < n ? old : n);
  free(p);
  return r;
}

int posix_memalign(void ** out, size_t a, size_t n) { init(); return real_posix_memalign(out, a, n); }
void * aligned_alloc(size_t a, size_t n) { init(); return real_aligned_alloc(a, n); }
void * memalign(size_t a, size_t n) { init(); return real_memalign(a, n); }

__attribute__((destructor)) static void drain(void)
{
  // Everything still quarantined at exit is verified too.
  pthread_mutex_lock(&qlock);
  inside = 1;
  while (qcount) release_oldest();
  pthread_mutex_unlock(&qlock);
  if (getenv("QCHECK_VERBOSE")) {
    char line[160];
    int n = snprintf(line, sizeof line, "[qcheck] exit: %zu blocks quarantined, %zu verified clean\n", total_frees, total_verified);
    if (write(2, line, n) < 0) {}
  }
}
