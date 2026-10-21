/* Opt-in diagnostic: names the thread that keeps CPython's GIL while another
 * thread is waiting for it (CPython 3.10 internal layout, read-only).
 *
 * A pthread that never calls the Python C API polls every millisecond:
 *   - gil.locked, gil.switch_number and gilstate.tstate_current (the holder),
 *   - interp->ceval.gil_drop_request (another thread has asked for the GIL).
 * A hold is reported when one thread state kept the GIL, without a switch, for
 * at least the threshold while a drop request was seen. The probe also reports
 * its own lateness: if the probe thread itself was not scheduled for the
 * threshold (CPU or machine contention), a hold overlapping it cannot be
 * attributed to the holder alone. The loop ends once the interpreter starts
 * finalizing, so it never reads a deleted interpreter state.
 *
 * Records (one per line, written with write(2)):
 *   hold <start_wall_ns> <end_wall_ns> <hold_ms> <thread_id> <switch_number>
 *   late <start_wall_ns> <end_wall_ns> <late_ms>
 * thread_id is PyThreadState.thread_id (= threading.Thread.ident).
 * It never changes interpreter state.
 */
#define Py_BUILD_CORE 1
#include <Python.h>
#include "internal/pycore_runtime.h"
#include "internal/pycore_interp.h"

#include <fcntl.h>
#include <pthread.h>
#include <sys/uio.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

/* The CPython version whose internal layout this library was compiled against. */
const char gil_probe_python_version[] = PY_VERSION;

static pthread_t probe_thread;
static int probe_running;
static int probe_fd = -1;
static long long probe_threshold_ns;
static long long probe_holds, probe_lates, probe_read_failures;

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

static void *probe_main(void *unused) {
    (void)unused;
    struct _gil_runtime_state *gil = &_PyRuntime.ceval.gil;
    PyInterpreterState *interp = _PyRuntime.interpreters.main;
    uintptr_t holder = 0;
    unsigned long holder_id = 0, holder_switch = 0;
    long long hold_start = 0, hold_start_wall = 0, last_poll = now_ns(CLOCK_MONOTONIC);
    int contended = 0;
    char line[160];
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
        int drop = interp ? __atomic_load_n(&interp->ceval.gil_drop_request._value, __ATOMIC_RELAXED) : 0;
        unsigned long tid = 0;
        if (tstate) {
            /* A thread state is freed after its thread drops the GIL on exit: read the id
             * through process_vm_readv so a stale pointer yields EFAULT, never a fault. */
            struct iovec local = {&tid, sizeof tid};
            struct iovec remote = {&((PyThreadState *)tstate)->thread_id, sizeof tid};
            if (process_vm_readv(getpid(), &local, 1, &remote, 1, 0) != (ssize_t)sizeof tid) { tstate = 0; probe_read_failures++; }
            /* Only trust the id if the holder did not change while it was read. */
            if (__atomic_load_n(&_PyRuntime.gilstate.tstate_current._value, __ATOMIC_ACQUIRE) != tstate) tstate = 0;
        }
        if (locked == 1 && tstate && tstate == holder && switches == holder_switch) {
            if (drop) contended = 1;
            continue;
        }
        if (holder && contended && mono - hold_start >= probe_threshold_ns) {
            probe_holds++;
            emit(line, snprintf(line, sizeof line, "hold %lld %lld %.3f %lu %lu\n",
                 hold_start_wall, wall, (mono - hold_start) / 1e6, holder_id, holder_switch), sizeof line);
        }
        holder = (locked == 1) ? tstate : 0;
        holder_id = tid;
        holder_switch = switches;
        hold_start = mono;
        hold_start_wall = wall;
        contended = holder && drop;
    }
    return NULL;
}

/* Returns 0 on success. Called with the GIL held (ctypes); the probe thread itself never takes it. */
int gil_probe_start(const char *path, long long threshold_ns) {
    if (__atomic_load_n(&probe_running, __ATOMIC_ACQUIRE) || !path || threshold_ns <= 0) return -1;
    probe_fd = open(path, O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
    if (probe_fd < 0) return -2;
    probe_threshold_ns = threshold_ns;
    probe_holds = probe_lates = probe_read_failures = 0;
    __atomic_store_n(&probe_running, 1, __ATOMIC_RELEASE);
    if (pthread_create(&probe_thread, NULL, probe_main, NULL) != 0) {
        __atomic_store_n(&probe_running, 0, __ATOMIC_RELEASE);
        close(probe_fd);
        probe_fd = -1;
        return -3;
    }
    return 0;
}

/* Stops and joins the probe thread; returns the number of hold records written. */
long long gil_probe_stop(void) {
    if (!__atomic_load_n(&probe_running, __ATOMIC_ACQUIRE)) return -1;
    __atomic_store_n(&probe_running, 0, __ATOMIC_RELEASE);
    pthread_join(probe_thread, NULL);
    close(probe_fd);
    probe_fd = -1;
    return probe_holds;
}

long long gil_probe_lates(void) { return probe_lates; }

/* Holder-id reads that failed (e.g. a sandbox denying process_vm_readv): non-zero means holds may be missing. */
long long gil_probe_read_failures(void) { return probe_read_failures; }
