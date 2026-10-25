/* Standalone diagnostic: detects pauses of this guest's vCPUs (Stage-A GIL-stall investigation).
 *
 * A separate process, never loaded into the executor. One thread per online CPU is pinned
 * to that CPU; every period it sleeps to an absolute deadline, then spins for a short
 * window reading CLOCK_MONOTONIC back to back. A gap between two consecutive reads of at
 * least gap_ms means the thread did not run for that long. Each gap is classified from
 * guest-side evidence only:
 *   guest_switch  - the thread's own voluntary/involuntary context-switch counters moved
 *                   (preempted or stopped by the guest kernel);
 *   guest_irq     - no switch, but its CPU's /proc/stat irq+softirq ticks grew by at least
 *                   half the gap (interrupt handling inside the guest);
 *   invisible     - no switch and no softirq/irq time on that CPU: the guest kept no trace of
 *                   the time. Candidates: the hypervisor descheduling (or slowly waking) the
 *                   vCPU, firmware (SMI), or - not excluded here - a hardirq/NMI handler or an
 *                   irq-disabled kernel section on that CPU: with tick-sampled accounting the
 *                   irq column stays 0 and hardirq time is charged to the interrupted task.
 *   migrated      - the thread was not on its CPU during the window (affinity violated).
 *   short_unresolved - no switch and no irq ticks, but shorter than resolve_ms: /proc/stat
 *                   counts irq time in 10 ms ticks, so a guest irq burst this short leaves no
 *                   trace and cannot be excluded. 'invisible' therefore needs >= resolve_ms.
 * It also records wake-ups from the absolute-deadline sleep that are at least late_ms late,
 * classified the same way from what happened during the sleep: runqueue (run-queue wait
 * covers half the lateness: guest CPU contention), guest_irq (irq+softirq ticks do),
 * stopped (more than one voluntary switch: the process was stopped), else invisible.
 * Late wakes are weaker evidence than gaps: the guest records less about a sleeping thread.
 * Every record carries that CPU's busy (user+nice+system) and idle (idle+iowait) tick deltas,
 * so an idle vCPU woken late can be told from a busy vCPU that stopped running.
 * Workers are phase-staggered (worker i starts i/ncpus of a period later), so the vCPUs never
 * all wake and spin at the same instant.
 * The kernel here has CONFIG_IRQ_TIME_ACCOUNTING off, so thread CPU time is charged for
 * irq time too; the irq/softirq tick delta is what separates guest_irq from invisible.
 * /proc/stat ticks are USER_HZ (10 ms), so guest irq bursts shorter than ~20 ms are not
 * resolved; a gap is classified guest_irq only when the ticks cover half of it.
 *
 * Output (one line per record, write(2) with O_APPEND):
 *   config <key=value>...
 *   gap <cpu> <tid> <start_wall_ns> <gap_ms> <d_cpu_ns> <d_nvcsw> <d_nivcsw> <d_irq_ticks> <d_softirq_ticks> <d_steal_ticks> <class> <d_busy_ticks> <d_idle_ticks>
 *   late <cpu> <tid> <deadline_wall_ns> <late_ms> <d_runqueue_wait_ns> <d_nvcsw> <d_irq_ticks> <d_softirq_ticks> <d_steal_ticks> <class> <d_busy_ticks> <d_idle_ticks>
 *   overrun <cpu> <tid> <wall_ns> <late_by_ms> <d_nvcsw> <d_nivcsw>
 *        (the deadline had already passed before the sleep began: the pause fell outside the
 *        measured intervals; with no own switch it is unexplained, never a 'late' record)
 *   beat <wall_ns> <cpu> <windows> <gaps> <lates> <max_gap_ms>
 *   summary <wall_ns> <pinned_cpus> <windows> <gaps> <invisible> <guest_switch> <guest_irq> <migrated> <lates> <max_gap_ms> <late_invisible> <max_late_ms> <covered_cpu_list>
 *
 * usage: vcpu_pause_detector --output FILE [--duration S] [--period-ms 20] [--spin-us 150]
 *                            [--gap-ms 2] [--late-ms 10] [--resolve-ms 30]
 * Stops after --duration seconds or on SIGINT/SIGTERM. Needs no privileges.
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

static volatile sig_atomic_t stop_requested;   /* written by the signal handler, read by all threads */
static int out_fd = -1;
static long long period_ns = 20000000LL, spin_ns = 150000LL, gap_ns = 2000000LL, late_ns = 10000000LL, resolve_ns = 30000000LL;

struct worker {
    int cpu, index, count, pinned;
    pthread_t thread;
    long long windows, gaps, lates, invisible, guest_switch, guest_irq, migrated, max_gap, late_invisible, max_late;
};

static long long now(clockid_t clock) {
    struct timespec ts;
    clock_gettime(clock, &ts);
    return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void emit(const char *line, int length) {
    if (out_fd >= 0 && length > 0) {
        ssize_t ignored = write(out_fd, line, (size_t)length);
        (void)ignored;
    }
}

struct ticks { unsigned long long irq, softirq, steal, busy, idle; };

/* irq, softirq, steal, busy and idle ticks of one CPU from /proc/stat; returns 0 on success. */
static int cpu_ticks(int cpu, struct ticks *t) {
    char buffer[16384], label[16];
    int fd = open("/proc/stat", O_RDONLY | O_CLOEXEC);
    if (fd < 0) return -1;
    ssize_t n = read(fd, buffer, sizeof buffer - 1);
    close(fd);
    if (n <= 0) return -1;
    buffer[n] = 0;
    snprintf(label, sizeof label, "\ncpu%d ", cpu);
    char *line = strstr(buffer, label);
    if (!line) return -1;
    unsigned long long user, nice, sys, idle, iowait;
    if (sscanf(line + strlen(label), "%llu %llu %llu %llu %llu %llu %llu %llu",
               &user, &nice, &sys, &idle, &iowait, &t->irq, &t->softirq, &t->steal) != 8) return -1;
    t->busy = user + nice + sys;
    t->idle = idle + iowait;
    return 0;
}

static long long runqueue_wait(void) {
    char path[64], text[128];
    snprintf(path, sizeof path, "/proc/self/task/%ld/schedstat", (long)syscall(SYS_gettid));
    int fd = open(path, O_RDONLY | O_CLOEXEC);
    if (fd < 0) return -1;
    ssize_t n = read(fd, text, sizeof text - 1);
    close(fd);
    if (n <= 0) return -1;
    text[n] = 0;
    unsigned long long run, wait;
    return sscanf(text, "%llu %llu", &run, &wait) == 2 ? (long long)wait : -1;
}

static void *run_worker(void *argument) {
    struct worker *w = argument;
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(w->cpu, &set);
    if (pthread_setaffinity_np(pthread_self(), sizeof set, &set) != 0) return NULL;
    w->pinned = 1;
    long tid = (long)syscall(SYS_gettid);
    char line[448];
    struct timespec deadline;
    clock_gettime(CLOCK_MONOTONIC, &deadline);
    /* phase stagger: worker i starts i/count of a period later than worker 0 */
    long long first = (long long)deadline.tv_sec * 1000000000LL + deadline.tv_nsec + period_ns * w->index / (w->count ? w->count : 1);
    deadline.tv_sec = first / 1000000000LL;
    deadline.tv_nsec = first % 1000000000LL;
    long long next_beat = now(CLOCK_MONOTONIC) + 10000000000LL;
    long long tick_ns = 1000000000LL / sysconf(_SC_CLK_TCK);
    struct ticks prev = {0, 0, 0, 0, 0};
    int prev_ok = 0;
    while (!stop_requested) {
        long long due = (long long)deadline.tv_sec * 1000000000LL + deadline.tv_nsec + period_ns;
        deadline.tv_sec = due / 1000000000LL;
        deadline.tv_nsec = due % 1000000000LL;
        long long wait_before = runqueue_wait();
        struct rusage slept0, slept1;
        getrusage(RUSAGE_THREAD, &slept0);
        long long called = now(CLOCK_MONOTONIC);
        while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, NULL) == EINTR && !stop_requested) {}
        if (stop_requested) break;
        long long woke = now(CLOCK_MONOTONIC);
        getrusage(RUSAGE_THREAD, &slept1);
        if (called >= due) {
            /* lateness accrued before sleeping (running, unmeasured): not a late wake */
            if (called - due >= late_ns)
                emit(line, snprintf(line, sizeof line, "overrun %d %ld %lld %.3f %ld %ld\n", w->cpu, tid,
                     now(CLOCK_REALTIME) - (now(CLOCK_MONOTONIC) - called), (called - due) / 1e6,
                     slept1.ru_nvcsw - slept0.ru_nvcsw, slept1.ru_nivcsw - slept0.ru_nivcsw));
        }
        /* irq/softirq/steal ticks now; compared with the previous window's start (before the sleep) */
        struct ticks t0 = {0, 0, 0, 0, 0}, t1 = {0, 0, 0, 0, 0};
        int ticks_ok = cpu_ticks(w->cpu, &t0) == 0;
        long long woke_wall = now(CLOCK_REALTIME) - (now(CLOCK_MONOTONIC) - woke);
        if (called < due && woke - due >= late_ns) {
            long long wait_after = runqueue_wait(), late = woke - due;
            long long rq = wait_before >= 0 && wait_after >= 0 ? wait_after - wait_before : -1;
            long nv = slept1.ru_nvcsw - slept0.ru_nvcsw;
            int ok = ticks_ok && prev_ok;
            long long dirq = ok ? (long long)(t0.irq - prev.irq) : -1, dsoft = ok ? (long long)(t0.softirq - prev.softirq) : -1;
            long long dsteal = ok ? (long long)(t0.steal - prev.steal) : -1;
            long long dbusy = ok ? (long long)(t0.busy - prev.busy) : -1, didle = ok ? (long long)(t0.idle - prev.idle) : -1;
            const char *kind;
            if (rq >= 0 && rq * 2 >= late) kind = "runqueue";
            else if (ok && (dirq + dsoft) * tick_ns * 2 >= late) kind = "guest_irq";
            else if (nv > 1) kind = "stopped";
            else if (late < resolve_ns) kind = "short_unresolved";
            else { kind = ok ? "invisible" : "invisible_unverified"; w->late_invisible++; }
            w->lates++;
            if (late > w->max_late) w->max_late = late;
            emit(line, snprintf(line, sizeof line, "late %d %ld %lld %.3f %lld %ld %lld %lld %lld %s %lld %lld\n", w->cpu, tid,
                 woke_wall - late, late / 1e6, rq, nv, dirq, dsoft, dsteal, kind, dbusy, didle));
        }
        /* The spin window: back-to-back clock reads; the largest gap is the pause. */
        struct rusage before, after;
        getrusage(RUSAGE_THREAD, &before);
        long long cpu0 = now(CLOCK_THREAD_CPUTIME_ID);
        long long start = now(CLOCK_MONOTONIC), previous = start, largest = 0, largest_at = start;
        int off_cpu = 0;
        while (previous - start < spin_ns) {
            long long t = now(CLOCK_MONOTONIC);
            if (t - previous > largest) { largest = t - previous; largest_at = previous; }
            previous = t;
            if (sched_getcpu() != w->cpu) off_cpu = 1;
        }
        long long cpu1 = now(CLOCK_THREAD_CPUTIME_ID);
        getrusage(RUSAGE_THREAD, &after);
        w->windows++;
        if (largest >= gap_ns) {
            ticks_ok = ticks_ok && cpu_ticks(w->cpu, &t1) == 0;
            long nv = after.ru_nvcsw - before.ru_nvcsw, niv = after.ru_nivcsw - before.ru_nivcsw;
            long long dirq = ticks_ok ? (long long)(t1.irq - t0.irq) : -1, dsoft = ticks_ok ? (long long)(t1.softirq - t0.softirq) : -1;
            long long dsteal = ticks_ok ? (long long)(t1.steal - t0.steal) : -1;
            long long dbusy = ticks_ok ? (long long)(t1.busy - t0.busy) : -1, didle = ticks_ok ? (long long)(t1.idle - t0.idle) : -1;
            const char *kind;
            if (off_cpu) { kind = "migrated"; w->migrated++; }
            else if (nv || niv) { kind = "guest_switch"; w->guest_switch++; }
            else if (ticks_ok && (dirq + dsoft) * tick_ns * 2 >= largest) { kind = "guest_irq"; w->guest_irq++; }
            else if (largest < resolve_ns) kind = "short_unresolved";
            else { kind = ticks_ok ? "invisible" : "invisible_unverified"; w->invisible++; }
            w->gaps++;
            if (largest > w->max_gap) w->max_gap = largest;
            emit(line, snprintf(line, sizeof line, "gap %d %ld %lld %.3f %lld %ld %ld %lld %lld %lld %s %lld %lld\n", w->cpu, tid,
                 now(CLOCK_REALTIME) - (now(CLOCK_MONOTONIC) - largest_at), largest / 1e6, cpu1 - cpu0, nv, niv,
                 dirq, dsoft, dsteal, kind, dbusy, didle));
        }
        prev = t0; prev_ok = ticks_ok;
        if (now(CLOCK_MONOTONIC) >= next_beat) {
            next_beat += 10000000000LL;
            emit(line, snprintf(line, sizeof line, "beat %lld %d %lld %lld %lld %.3f\n", now(CLOCK_REALTIME), w->cpu,
                 w->windows, w->gaps, w->lates, w->max_gap / 1e6));
        }
        /* a long pause pushes the schedule: never try to catch up with a burst of windows */
        long long late_by = now(CLOCK_MONOTONIC) - ((long long)deadline.tv_sec * 1000000000LL + deadline.tv_nsec);
        if (late_by > period_ns) clock_gettime(CLOCK_MONOTONIC, &deadline);   /* the next sleep starts on time */
    }
    return NULL;
}

static void on_signal(int signo) { (void)signo; stop_requested = 1; }

static long long arg_value(const char *text, long long scale) {
    char *end;
    double value = strtod(text, &end);
    if (*end || value <= 0) { fprintf(stderr, "bad value: %s\n", text); exit(2); }
    return (long long)(value * scale);
}

int main(int argc, char **argv) {
    const char *output = NULL;
    double duration = 0;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--output") && i + 1 < argc) output = argv[++i];
        else if (!strcmp(argv[i], "--duration") && i + 1 < argc) duration = arg_value(argv[++i], 1000) / 1e3;
        else if (!strcmp(argv[i], "--period-ms") && i + 1 < argc) period_ns = arg_value(argv[++i], 1000000);
        else if (!strcmp(argv[i], "--spin-us") && i + 1 < argc) spin_ns = arg_value(argv[++i], 1000);
        else if (!strcmp(argv[i], "--gap-ms") && i + 1 < argc) gap_ns = arg_value(argv[++i], 1000000);
        else if (!strcmp(argv[i], "--late-ms") && i + 1 < argc) late_ns = arg_value(argv[++i], 1000000);
        else if (!strcmp(argv[i], "--resolve-ms") && i + 1 < argc) resolve_ns = arg_value(argv[++i], 1000000);
        else { fprintf(stderr, "usage: %s --output FILE [--duration S] [--period-ms N] [--spin-us N] [--gap-ms N] [--late-ms N]\n", argv[0]); return 2; }
    }
    if (!output || spin_ns >= period_ns || gap_ns < 100000) { fprintf(stderr, "need --output; spin < period; gap >= 0.1 ms\n"); return 2; }
    out_fd = open(output, O_WRONLY | O_CREAT | O_TRUNC | O_APPEND | O_CLOEXEC, 0644);
    if (out_fd < 0) { perror(output); return 1; }
    struct sigaction action;
    memset(&action, 0, sizeof action);
    action.sa_handler = on_signal;
    sigaction(SIGINT, &action, NULL);
    sigaction(SIGTERM, &action, NULL);
    cpu_set_t allowed;
    sched_getaffinity(0, sizeof allowed, &allowed);
    int cpus = 0;
    struct worker workers[CPU_SETSIZE];
    char line[320];
    int length = snprintf(line, sizeof line, "config start_wall_ns=%lld pid=%d period_ms=%.3f spin_us=%.1f gap_ms=%.3f late_ms=%.3f resolve_ms=%.3f duration_s=%.1f clk_tck=%ld\n",
                          now(CLOCK_REALTIME), getpid(), period_ns / 1e6, spin_ns / 1e3, gap_ns / 1e6, late_ns / 1e6, resolve_ns / 1e6, duration, sysconf(_SC_CLK_TCK));
    emit(line, length);
    int count = CPU_COUNT(&allowed);
    for (int cpu = 0; cpu < CPU_SETSIZE; cpu++) {
        if (!CPU_ISSET(cpu, &allowed)) continue;
        memset(&workers[cpus], 0, sizeof workers[cpus]);
        workers[cpus].cpu = cpu;
        workers[cpus].index = cpus;
        workers[cpus].count = count;
        if (pthread_create(&workers[cpus].thread, NULL, run_worker, &workers[cpus]) == 0) cpus++;
    }
    long long end = duration > 0 ? now(CLOCK_MONOTONIC) + (long long)(duration * 1e9) : 0;
    while (!stop_requested && (!end || now(CLOCK_MONOTONIC) < end)) {
        struct timespec pause = {0, 100000000};
        nanosleep(&pause, NULL);
    }
    stop_requested = 1;
    long long windows = 0, gaps = 0, invisible = 0, guest_switch = 0, guest_irq = 0, migrated = 0, lates = 0, max_gap = 0;
    long long late_invisible = 0, max_late = 0;
    int pinned = 0;
    char covered[256] = "";
    for (int i = 0; i < cpus; i++) {
        pthread_join(workers[i].thread, NULL);
        if (workers[i].pinned) {
            pinned++;
            size_t used = strlen(covered);
            snprintf(covered + used, sizeof covered - used, "%s%d", used ? "," : "", workers[i].cpu);
        }
        windows += workers[i].windows; gaps += workers[i].gaps; invisible += workers[i].invisible;
        guest_switch += workers[i].guest_switch; guest_irq += workers[i].guest_irq; migrated += workers[i].migrated;
        lates += workers[i].lates;
        if (workers[i].max_gap > max_gap) max_gap = workers[i].max_gap;
        late_invisible += workers[i].late_invisible;
        if (workers[i].max_late > max_late) max_late = workers[i].max_late;
    }
    emit(line, snprintf(line, sizeof line, "summary %lld %d %lld %lld %lld %lld %lld %lld %lld %.3f %lld %.3f %s\n", now(CLOCK_REALTIME), pinned,
         windows, gaps, invisible, guest_switch, guest_irq, migrated, lates, max_gap / 1e6, late_invisible, max_late / 1e6,
         covered[0] ? covered : "-"));
    close(out_fd);
    return 0;
}
