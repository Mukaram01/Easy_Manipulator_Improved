/* Opt-in diagnostic: attributes long holds of CPython's GIL (CPython 3.10, read-only).
 *
 * What the probe thread reads without the GIL, and how
 *   A pthread that never calls the Python C API polls every millisecond. It reads only
 *   fields of the static _PyRuntime structure, with atomic loads, as CPython's own
 *   threads do without the GIL: ceval.gil.locked, ceval.gil.switch_number,
 *   gilstate.tstate_current and _finalizing. The thread-state pointer is used as an
 *   opaque value: the probe never dereferences an interpreter state, a thread state or
 *   a frame. A hold is one thread state keeping the GIL, with no switch, for at least
 *   the threshold. The Python side always runs a sampler thread that asks for the GIL
 *   every 20 ms, so a hold this long is contended; the sampler records its own waits.
 *
 * Holder identity
 *   Python threads register (thread-state pointer, kernel tid) themselves while they
 *   hold the GIL (gil_probe_register). A holder that never registered is tid -1.
 *
 * During a long hold, and only then, for the holder's kernel thread
 *   - /proc/self/task/<tid>/stat and schedstat at the threshold crossing and at the end
 *     (run state, user/system ticks, faults, on-CPU and run-queue time);
 *   - /proc/self/task/<tid>/syscall every 5 ms: "running", or the syscall it is blocked
 *     in with its user-space PC;
 *   - optional native PC samples (pc_enable): one signal at a time, sent only while the
 *     holder is running (not in a syscall), at most every 2 ms. The handler copies the
 *     interrupted RIP/RSP and up to 4 KiB of the holder's own stack with
 *     process_vm_readv (fault-safe): no unwinding, no allocation, no Python. Stack words
 *     pointing into executable mappings are reported as candidate return addresses
 *     (a heuristic: stale words can appear). The handler stays installed until exit and
 *     ignores any signal it did not request, so a late delivery is never fatal.
 *
 * Under the GIL (ctypes.PyDLL), for the Python sampler
 *   gil_probe_snapshot() appends every thread's Python frames, the live thread-state ->
 *   thread ident map and the current switch number in one native call, so no thread
 *   runs while they are read; the map confirms (or corrects) a registration.
 *
 * Records (one per line, written with write(2)):
 *   hold <start_wall_ns> <end_wall_ns> <hold_ms> <tstate> <switch_number> <tid>
 *   cpu  <hold_start_wall_ns> <tid> <begin|end> <state> <utime> <stime> <minflt> <majflt> <run_ns> <wait_ns> <slices>
 *   sys  <hold_start_wall_ns> <wall_ns> <first line of /proc/self/task/<tid>/syscall>
 *   pc   <hold_start_wall_ns> <sent_wall_ns> <delivery_us> <tid> <rip> <rsp> <stack_bytes> <n> <addr>...
 *   late <start_wall_ns> <end_wall_ns> <late_ms>
 */
#define Py_BUILD_CORE 1
#include <Python.h>
#include <frameobject.h>
#include "internal/pycore_runtime.h"

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/syscall.h>
#include <sys/uio.h>
#include <time.h>
#include <ucontext.h>
#include <unistd.h>

/* The CPython version whose internal layout this library was compiled against. */
const char gil_probe_python_version[] = PY_VERSION;

#define MAX_THREADS 256
#define SYS_PERIOD_NS 5000000LL
#define PC_PERIOD_NS 2000000LL
#define PC_TIMEOUT_NS 50000000LL
#define STACK_BYTES 4096
#define STACK_CHUNK 512
#define MAX_RANGES 1024
#define MAX_CANDIDATES 32

static pthread_t probe_thread;
static int probe_running;
static int probe_fd = -1;
static long long probe_threshold_ns;
static long long probe_holds, probe_lates, probe_unmapped, probe_proc_failures;
static long long pc_sent, pc_received, pc_lost, pc_send_failures, pc_skipped_in_syscall;
static int pc_enabled, pc_signal, pc_installed;
static pid_t probe_pid;

static struct { uintptr_t tstate; int tid; } registry[MAX_THREADS];
static int registry_count;

/* One outstanding PC request at a time; written by the handler, read by the probe. */
static unsigned long pc_pending, pc_done;
static struct {
    int tid;
    long long received_ns;
    uintptr_t rip, rsp;
    ssize_t bytes;
    unsigned char stack[STACK_BYTES];
} pc_slot;

static struct { uintptr_t start, end; } ranges[MAX_RANGES];
static int range_count;
static char maps_buffer[1 << 19];

static long long now_ns(clockid_t clock) {
    struct timespec ts;
    clock_gettime(clock, &ts);
    return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void emit(const char *line, int length, size_t size) {
    if (length > (int)size - 1) length = (int)size - 1;
    if (probe_fd >= 0 && length > 0) {
        ssize_t ignored = write(probe_fd, line, (size_t)length);
        (void)ignored;
    }
}

static ssize_t read_file(const char *path, char *buffer, size_t size) {
    int fd = open(path, O_RDONLY | O_CLOEXEC);
    if (fd < 0) return -1;
    size_t used = 0;
    while (used + 1 < size) {
        ssize_t n = read(fd, buffer + used, size - 1 - used);
        if (n <= 0) break;
        used += (size_t)n;
    }
    close(fd);
    buffer[used] = 0;
    return (ssize_t)used;
}

static int lookup_tid(uintptr_t tstate) {
    int count = __atomic_load_n(&registry_count, __ATOMIC_ACQUIRE);
    for (int i = count - 1; i >= 0; i--)
        if (__atomic_load_n(&registry[i].tstate, __ATOMIC_ACQUIRE) == tstate)
            return __atomic_load_n(&registry[i].tid, __ATOMIC_RELAXED);
    return -1;
}

struct cpu_sample {
    int ok;
    char state;
    unsigned long long utime, stime, minflt, majflt, run_ns, wait_ns, slices;
};

static void read_cpu(int tid, struct cpu_sample *sample) {
    char path[64], text[1024];
    memset(sample, 0, sizeof *sample);
    snprintf(path, sizeof path, "/proc/self/task/%d/stat", tid);
    if (read_file(path, text, sizeof text) <= 0) { probe_proc_failures++; return; }
    char *rest = strrchr(text, ')');
    if (!rest) { probe_proc_failures++; return; }
    /* fields after ") ": state ppid pgrp session tty tpgid flags minflt cminflt majflt cmajflt utime stime */
    unsigned long long skip;
    if (sscanf(rest + 1, " %c %llu %llu %llu %llu %llu %llu %llu %llu %llu %llu %llu %llu",
               &sample->state, &skip, &skip, &skip, &skip, &skip, &skip, &sample->minflt, &skip,
               &sample->majflt, &skip, &sample->utime, &sample->stime) != 13) { probe_proc_failures++; return; }
    snprintf(path, sizeof path, "/proc/self/task/%d/schedstat", tid);
    if (read_file(path, text, sizeof text) <= 0 ||
        sscanf(text, "%llu %llu %llu", &sample->run_ns, &sample->wait_ns, &sample->slices) != 3) {
        probe_proc_failures++; return;
    }
    sample->ok = 1;
}

static void emit_cpu(long long hold_start_wall, int tid, const char *phase, const struct cpu_sample *s) {
    char line[256];
    if (!s->ok) {
        emit(line, snprintf(line, sizeof line, "cpu %lld %d %s unavailable\n", hold_start_wall, tid, phase), sizeof line);
        return;
    }
    emit(line, snprintf(line, sizeof line, "cpu %lld %d %s %c %llu %llu %llu %llu %llu %llu %llu\n",
         hold_start_wall, tid, phase, s->state, s->utime, s->stime, s->minflt, s->majflt,
         s->run_ns, s->wait_ns, s->slices), sizeof line);
}

/* Returns 1 when the thread is running (not inside a syscall), 0 otherwise. */
static int sample_syscall(int tid, long long hold_start_wall, long long wall) {
    char path[64], text[512], line[640];
    snprintf(path, sizeof path, "/proc/self/task/%d/syscall", tid);
    if (read_file(path, text, sizeof text) <= 0) {
        probe_proc_failures++;
        emit(line, snprintf(line, sizeof line, "sys %lld %lld unavailable\n", hold_start_wall, wall), sizeof line);
        return 0;
    }
    char *newline = strchr(text, '\n');
    if (newline) *newline = 0;
    emit(line, snprintf(line, sizeof line, "sys %lld %lld %s\n", hold_start_wall, wall, text), sizeof line);
    return strncmp(text, "running", 7) == 0;
}

static void load_exec_ranges(void) {
    range_count = 0;
    if (read_file("/proc/self/maps", maps_buffer, sizeof maps_buffer) <= 0) { probe_proc_failures++; return; }
    for (char *line = maps_buffer; line && *line && range_count < MAX_RANGES;) {
        char *next = strchr(line, '\n');
        if (next) *next++ = 0;
        unsigned long start, end;
        char perms[8];
        if (sscanf(line, "%lx-%lx %7s", &start, &end, perms) == 3 && perms[2] == 'x') {
            ranges[range_count].start = start;
            ranges[range_count].end = end;
            range_count++;
        }
        line = next;
    }
}

static int executable(uintptr_t address) {
    for (int i = 0; i < range_count; i++)
        if (address >= ranges[i].start && address < ranges[i].end) return 1;
    return 0;
}

/* Async-signal-safe: atomics, registers and a fault-safe copy of this thread's stack. */
static void pc_handler(int signo, siginfo_t *info, void *context) {
    (void)signo;
    int saved_errno = errno;
    unsigned long want = __atomic_load_n(&pc_pending, __ATOMIC_ACQUIRE);
    if (info && info->si_code == SI_TKILL && info->si_pid == probe_pid && want &&
        __atomic_load_n(&pc_done, __ATOMIC_ACQUIRE) != want) {
        ucontext_t *uc = (ucontext_t *)context;
        pc_slot.tid = (int)syscall(SYS_gettid);
        pc_slot.received_ns = now_ns(CLOCK_MONOTONIC);
        pc_slot.rip = (uintptr_t)uc->uc_mcontext.gregs[REG_RIP];
        pc_slot.rsp = (uintptr_t)uc->uc_mcontext.gregs[REG_RSP];
        struct iovec local[STACK_BYTES / STACK_CHUNK], remote[STACK_BYTES / STACK_CHUNK];
        for (int i = 0; i < STACK_BYTES / STACK_CHUNK; i++) {
            local[i].iov_base = pc_slot.stack + i * STACK_CHUNK;
            local[i].iov_len = STACK_CHUNK;
            remote[i].iov_base = (void *)(pc_slot.rsp + (uintptr_t)(i * STACK_CHUNK));
            remote[i].iov_len = STACK_CHUNK;
        }
        /* A chunk past the end of the stack mapping stops the copy there (partial read). */
        ssize_t n = process_vm_readv(probe_pid, local, STACK_BYTES / STACK_CHUNK, remote, STACK_BYTES / STACK_CHUNK, 0);
        pc_slot.bytes = n < 0 ? 0 : n;
        __atomic_store_n(&pc_done, want, __ATOMIC_RELEASE);
    }
    errno = saved_errno;
}

static void emit_pc(long long hold_start_wall, long long sent_wall, long long sent_mono, int tid) {
    char line[1024];
    int length = snprintf(line, sizeof line, "pc %lld %lld %lld %d %#lx %#lx %zd", hold_start_wall, sent_wall,
                          (pc_slot.received_ns - sent_mono) / 1000, tid, (unsigned long)pc_slot.rip,
                          (unsigned long)pc_slot.rsp, pc_slot.bytes);
    uintptr_t candidates[MAX_CANDIDATES];
    int count = 0;
    for (ssize_t offset = 0; offset + (ssize_t)sizeof(uintptr_t) <= pc_slot.bytes && count < MAX_CANDIDATES;
         offset += (ssize_t)sizeof(uintptr_t)) {
        uintptr_t word;
        memcpy(&word, pc_slot.stack + offset, sizeof word);
        if (executable(word)) candidates[count++] = word;
    }
    length += snprintf(line + length, sizeof line - (size_t)length, " %d", count);
    for (int i = 0; i < count && length < (int)sizeof line - 24; i++)
        length += snprintf(line + length, sizeof line - (size_t)length, " %#lx", (unsigned long)candidates[i]);
    length += snprintf(line + length, sizeof line - (size_t)length, "\n");
    emit(line, length, sizeof line);
}

static void *probe_main(void *unused) {
    (void)unused;
    struct _gil_runtime_state *gil = &_PyRuntime.ceval.gil;
    uintptr_t holder = 0;
    unsigned long holder_switch = 0, pc_sequence = 0;
    long long hold_start = 0, hold_start_wall = 0, last_poll = now_ns(CLOCK_MONOTONIC);
    long long next_sys = 0, next_pc = 0, pc_sent_mono = 0, pc_sent_wall = 0;
    int long_hold = 0, tid = -1, running = 0, outstanding = 0;
    struct cpu_sample begin;
    char line[256];
    struct timespec period = {0, 1000000};
    while (__atomic_load_n(&probe_running, __ATOMIC_ACQUIRE)) {
        nanosleep(&period, NULL);
        if (__atomic_load_n(&_PyRuntime._finalizing._value, __ATOMIC_ACQUIRE)) break;
        long long mono = now_ns(CLOCK_MONOTONIC), wall = now_ns(CLOCK_REALTIME);
        if (mono - last_poll >= probe_threshold_ns) {
            probe_lates++;
            emit(line, snprintf(line, sizeof line, "late %lld %lld %.3f\n",
                 wall - (mono - last_poll), wall, (mono - last_poll) / 1e6), sizeof line);
        }
        last_poll = mono;
        int locked = __atomic_load_n(&gil->locked._value, __ATOMIC_ACQUIRE);
        unsigned long switches = __atomic_load_n(&gil->switch_number, __ATOMIC_RELAXED);
        uintptr_t tstate = __atomic_load_n(&_PyRuntime.gilstate.tstate_current._value, __ATOMIC_ACQUIRE);
        if (outstanding && __atomic_load_n(&pc_done, __ATOMIC_ACQUIRE) == pc_sequence) {
            outstanding = 0;
            if (pc_slot.tid == tid && long_hold) { pc_received++; emit_pc(hold_start_wall, pc_sent_wall, pc_sent_mono, tid); }
            else pc_lost++;
        } else if (outstanding && mono - pc_sent_mono > PC_TIMEOUT_NS) {
            outstanding = 0;
            pc_lost++;
            __atomic_store_n(&pc_pending, 0, __ATOMIC_RELEASE);   /* a late delivery is then ignored */
        }
        if (locked == 1 && tstate && tstate == holder && switches == holder_switch) {
            if (!long_hold && mono - hold_start >= probe_threshold_ns) {
                long_hold = 1;
                tid = lookup_tid(holder);
                if (tid > 0) read_cpu(tid, &begin); else memset(&begin, 0, sizeof begin);
                next_sys = mono;
                next_pc = mono;
                if (pc_enabled) load_exec_ranges();
            }
            if (long_hold && tid > 0) {
                if (mono >= next_sys) { running = sample_syscall(tid, hold_start_wall, wall); next_sys = mono + SYS_PERIOD_NS; }
                if (pc_enabled && !outstanding && mono >= next_pc) {
                    next_pc = mono + PC_PERIOD_NS;
                    if (!running) pc_skipped_in_syscall++;
                    else {
                        pc_sequence++;
                        __atomic_store_n(&pc_pending, pc_sequence, __ATOMIC_RELEASE);
                        pc_sent_mono = now_ns(CLOCK_MONOTONIC);
                        pc_sent_wall = now_ns(CLOCK_REALTIME);
                        if (syscall(SYS_tgkill, probe_pid, tid, pc_signal) == 0) { outstanding = 1; pc_sent++; }
                        else { pc_send_failures++; __atomic_store_n(&pc_pending, 0, __ATOMIC_RELEASE); }
                    }
                }
            }
            continue;
        }
        if (long_hold) {
            probe_holds++;
            if (tid <= 0) probe_unmapped++;
            emit(line, snprintf(line, sizeof line, "hold %lld %lld %.3f %#lx %lu %d\n", hold_start_wall, wall,
                 (mono - hold_start) / 1e6, (unsigned long)holder, holder_switch, tid), sizeof line);
            if (tid > 0) {
                struct cpu_sample end;
                read_cpu(tid, &end);
                emit_cpu(hold_start_wall, tid, "begin", &begin);
                emit_cpu(hold_start_wall, tid, "end", &end);
            }
        }
        holder = (locked == 1) ? tstate : 0;
        holder_switch = switches;
        hold_start = mono;
        hold_start_wall = wall;
        long_hold = 0;
        tid = -1;
        running = 0;
    }
    return NULL;
}

/* Called with the GIL held, from the registering thread itself (ctypes.PyDLL). */
int gil_probe_register(unsigned long tstate, int tid) {
    if (!tstate || tid <= 0) return -1;
    int count = __atomic_load_n(&registry_count, __ATOMIC_ACQUIRE);
    for (int i = 0; i < count; i++)
        if (__atomic_load_n(&registry[i].tstate, __ATOMIC_ACQUIRE) == tstate) {
            __atomic_store_n(&registry[i].tid, tid, __ATOMIC_RELEASE);   /* a reused thread state */
            return 0;
        }
    if (count >= MAX_THREADS) return -2;
    __atomic_store_n(&registry[count].tid, tid, __ATOMIC_RELAXED);
    __atomic_store_n(&registry[count].tstate, (uintptr_t)tstate, __ATOMIC_RELEASE);
    __atomic_store_n(&registry_count, count + 1, __ATOMIC_RELEASE);
    return 0;
}

static int install_pc_handler(void) {
    if (pc_installed) return 0;
    int signo = SIGRTMAX - 3;
    struct sigaction old;
    if (sigaction(signo, NULL, &old) != 0) return -4;
    if (old.sa_handler != SIG_DFL) return -5;   /* the signal is in use: never take it over */
    struct sigaction action;
    memset(&action, 0, sizeof action);
    action.sa_sigaction = pc_handler;
    action.sa_flags = SA_SIGINFO | SA_RESTART;
    sigemptyset(&action.sa_mask);
    if (sigaction(signo, &action, NULL) != 0) return -6;
    pc_signal = signo;
    pc_installed = 1;
    return 0;
}

/* Returns 0 on success. Called with the GIL held (ctypes); the probe thread itself never takes it. */
int gil_probe_start(const char *path, long long threshold_ns, int pc_enable) {
    if (__atomic_load_n(&probe_running, __ATOMIC_ACQUIRE) || !path || threshold_ns <= 0) return -1;
    probe_pid = getpid();
    __atomic_store_n(&registry_count, 0, __ATOMIC_RELEASE);   /* registrations belong to this start only */
    if (pc_enable) {
        int rc = install_pc_handler();
        if (rc) return rc;
    }
    probe_fd = open(path, O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
    if (probe_fd < 0) return -2;
    probe_threshold_ns = threshold_ns;
    probe_holds = probe_lates = probe_unmapped = probe_proc_failures = 0;
    pc_sent = pc_received = pc_lost = pc_send_failures = pc_skipped_in_syscall = 0;
    pc_enabled = pc_enable ? 1 : 0;
    __atomic_store_n(&probe_running, 1, __ATOMIC_RELEASE);
    if (pthread_create(&probe_thread, NULL, probe_main, NULL) != 0) {
        __atomic_store_n(&probe_running, 0, __ATOMIC_RELEASE);
        close(probe_fd);
        probe_fd = -1;
        return -3;
    }
    pthread_setname_np(probe_thread, "gil_probe");   /* names it in /proc and process_watch records */
    return 0;
}

/* Stops and joins the probe thread; returns the number of hold records written. */
long long gil_probe_stop(void) {
    if (!__atomic_load_n(&probe_running, __ATOMIC_ACQUIRE)) return -1;
    __atomic_store_n(&probe_running, 0, __ATOMIC_RELEASE);
    pthread_join(probe_thread, NULL);
    __atomic_store_n(&pc_pending, 0, __ATOMIC_RELEASE);   /* the handler stays, inert */
    close(probe_fd);
    probe_fd = -1;
    return probe_holds;
}

/* Counters, in a fixed order, for the Python summary. */
int gil_probe_counters(long long *out, int size) {
    long long values[] = {probe_holds, probe_lates, probe_unmapped, probe_proc_failures,
                          pc_sent, pc_received, pc_lost, pc_send_failures, pc_skipped_in_syscall};
    int n = (int)(sizeof values / sizeof values[0]);
    for (int i = 0; i < n && i < size; i++) out[i] = values[i];
    return n;
}

int gil_probe_pc_signal(void) { return pc_installed ? pc_signal : 0; }

/* Called with the GIL held (ctypes.PyDLL) by the Python sampler. Appends one tuple
 * (switch_number, wall_ns, monotonic_ns, {thread ident: [(filename, name, firstlineno,
 * lasti_bytes, lineno), ... innermost first]}, {thread-state pointer: thread ident}) to
 * out. No bytecode runs and the GIL is not released during the call, so no thread can
 * move while its frames are read. The thread-state list is walked exactly as CPython's
 * own _PyThread_CurrentFrames walks it: GIL held, under the runtime's head lock, and
 * only into a fixed C array (no allocation while the lock is held). */
#define MAX_TSTATES 512
int gil_probe_snapshot(PyObject *out, int max_depth) {
    if (!PyList_Check(out) || max_depth <= 0) {
        PyErr_SetString(PyExc_TypeError, "gil_probe_snapshot needs a list and a positive depth");
        return -1;
    }
    unsigned long switches = __atomic_load_n(&_PyRuntime.ceval.gil.switch_number, __ATOMIC_RELAXED);
    long long wall = now_ns(CLOCK_REALTIME), mono = now_ns(CLOCK_MONOTONIC);
    static uintptr_t live_tstate[MAX_TSTATES];
    static unsigned long live_ident[MAX_TSTATES];
    int live = 0;
    PyThread_acquire_lock(_PyRuntime.interpreters.mutex, WAIT_LOCK);
    for (PyThreadState *t = PyInterpreterState_ThreadHead(PyInterpreterState_Main()); t && live < MAX_TSTATES;
         t = PyThreadState_Next(t)) {
        live_tstate[live] = (uintptr_t)t;
        live_ident[live] = t->thread_id;
        live++;
    }
    PyThread_release_lock(_PyRuntime.interpreters.mutex);
    PyObject *tstates = PyDict_New();
    if (!tstates) return -1;
    for (int i = 0; i < live; i++) {
        PyObject *key = PyLong_FromUnsignedLong((unsigned long)live_tstate[i]);
        PyObject *value = PyLong_FromUnsignedLong(live_ident[i]);
        int rc = (key && value) ? PyDict_SetItem(tstates, key, value) : -1;
        Py_XDECREF(key);
        Py_XDECREF(value);
        if (rc < 0) { Py_DECREF(tstates); return -1; }
    }
    PyObject *frames = _PyThread_CurrentFrames();
    if (!frames) { Py_DECREF(tstates); return -1; }
    PyObject *threads = PyDict_New();
    if (!threads) { Py_DECREF(frames); Py_DECREF(tstates); return -1; }
    Py_ssize_t position = 0;
    PyObject *ident, *top;
    while (PyDict_Next(frames, &position, &ident, &top)) {
        PyObject *stack = PyList_New(0);
        if (!stack) goto fail;
        PyFrameObject *frame = (PyFrameObject *)top;
        Py_INCREF(frame);
        for (int depth = 0; frame && depth < max_depth; depth++) {
            PyCodeObject *code = PyFrame_GetCode(frame);
            PyObject *lasti = PyObject_GetAttrString((PyObject *)frame, "f_lasti");
            PyObject *item = (code && lasti) ? Py_BuildValue("(OOiOi)", code->co_filename, code->co_name,
                                                             code->co_firstlineno, lasti, PyFrame_GetLineNumber(frame)) : NULL;
            Py_XDECREF(code);
            Py_XDECREF(lasti);
            if (!item || PyList_Append(stack, item) < 0) { Py_XDECREF(item); Py_DECREF(frame); Py_DECREF(stack); goto fail; }
            Py_DECREF(item);
            PyFrameObject *back = PyFrame_GetBack(frame);
            Py_DECREF(frame);
            frame = back;
        }
        Py_XDECREF(frame);
        if (PyDict_SetItem(threads, ident, stack) < 0) { Py_DECREF(stack); goto fail; }
        Py_DECREF(stack);
    }
    Py_DECREF(frames);
    PyObject *record = Py_BuildValue("(kLLNN)", switches, wall, mono, threads, tstates);
    if (!record) return -1;
    int rc = PyList_Append(out, record);
    Py_DECREF(record);
    return rc;
fail:
    Py_DECREF(threads);
    Py_DECREF(frames);
    Py_DECREF(tstates);
    return -1;
}

/* Test only (never called by the executor): spins without releasing the GIL when called
 * through ctypes.PyDLL, so attribution and native PC samples can be checked offline. */
__attribute__((noinline)) unsigned long long gil_probe_selftest_spin(long long duration_ns) {
    volatile unsigned long long count = 0;
    long long end = now_ns(CLOCK_MONOTONIC) + duration_ns;
    while (now_ns(CLOCK_MONOTONIC) < end) count++;
    return count;
}
