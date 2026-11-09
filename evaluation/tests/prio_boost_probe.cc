// Probe for the A2 priority-inversion guard in src/PointCloudMapping.cc.
//
// Performs the same scheduler syscalls as the dense thread, on a worker thread of
// its own, and reports whether each one succeeded:
//   1. ApplyLowPriority():  pthread_setschedparam(SCHED_IDLE)
//   2. PrioBoost ctor:      pthread_setschedparam(SCHED_OTHER)   <- the call that can fail
//   3. PrioBoost dtor:      pthread_setschedparam(SCHED_IDLE)
// Linux refuses step 2 for an unprivileged thread coming from SCHED_IDLE unless it
// has CAP_SYS_NICE or RLIMIT_NICE >= 20 (kernel/sched/core.c: "Treat SCHED_IDLE as
// nice 20. Only allow a switch to SCHED_NORMAL if the RLIMIT_NICE would normally
// permit it"). Docker drops CAP_SYS_NICE by default.
//
// Build:  g++ -O2 -std=c++17 -pthread prio_boost_probe.cc -o prio_boost_probe
// Run:    ./prio_boost_probe            (exit status 0: boost works, 1: boost fails)

#include <pthread.h>
#include <sched.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <thread>

static const char *policyName(int p)
{
    switch (p)
    {
        case SCHED_OTHER: return "SCHED_OTHER";
        case SCHED_IDLE:  return "SCHED_IDLE";
        case SCHED_BATCH: return "SCHED_BATCH";
        case SCHED_FIFO:  return "SCHED_FIFO";
        case SCHED_RR:    return "SCHED_RR";
        default:          return "?";
    }
}

static std::string procStatusField(const char *key)
{
    std::ifstream f("/proc/self/status");
    std::string line;
    const std::string k = std::string(key) + ":";
    while (std::getline(f, line))
        if (line.compare(0, k.size(), k) == 0) return line.substr(k.size());
    return "?";
}

int main()
{
    struct rlimit rl;
    getrlimit(RLIMIT_NICE, &rl);
    std::printf("uid=%d euid=%d  RLIMIT_NICE soft=%lu hard=%lu (needs >= 20 to leave SCHED_IDLE)\n",
                static_cast<int>(getuid()), static_cast<int>(geteuid()),
                static_cast<unsigned long>(rl.rlim_cur), static_cast<unsigned long>(rl.rlim_max));
    std::printf("CapEff:%s CapBnd:%s CapAmb:%s\n", procStatusField("CapEff").c_str(),
                procStatusField("CapBnd").c_str(), procStatusField("CapAmb").c_str());

    int exitCode = 0;
    std::thread dense([&exitCode] {
        sched_param sp;
        sp.sched_priority = 0;

        // 1. ApplyLowPriority()
        int rc = pthread_setschedparam(pthread_self(), SCHED_IDLE, &sp);
        std::printf("1. ApplyLowPriority: SCHED_IDLE            -> %s (policy now %s)\n",
                    rc == 0 ? "ok" : std::strerror(rc), policyName(sched_getscheduler(0)));
        if (rc != 0)
        {
            const int n = setpriority(PRIO_PROCESS, static_cast<id_t>(syscall(SYS_gettid)), 10);
            std::printf("   fallback nice +10 -> %s\n", n == 0 ? "ok" : std::strerror(errno));
            exitCode = 2;
            return;
        }

        // 2. PrioBoost ctor, repeated like the real thread does (twice per keyframe plus
        //    once per wait_for wake-up, about every 50 ms).
        int ok = 0, fail = 0, firstErr = 0;
        for (int i = 0; i < 1000; ++i)
        {
            rc = pthread_setschedparam(pthread_self(), SCHED_OTHER, &sp);
            if (rc == 0)
            {
                ++ok;
                // 3. PrioBoost dtor
                pthread_setschedparam(pthread_self(), SCHED_IDLE, &sp);
            }
            else
            {
                ++fail;
                if (!firstErr) firstErr = rc;
            }
        }
        std::printf("2. PrioBoost ctor:   SCHED_OTHER x1000     -> %d ok, %d failed%s%s (policy now %s)\n",
                    ok, fail, fail ? ", first error: " : "", fail ? std::strerror(firstErr) : "",
                    policyName(sched_getscheduler(0)));
        exitCode = (fail == 0) ? 0 : 1;
    });
    dense.join();

    std::printf("VERDICT: %s\n", exitCode == 0 ? "boost WORKS (no priority inversion)"
                              : exitCode == 1 ? "boost FAILS (dense thread would hold mMutexQueue/mMutexCloud at SCHED_IDLE)"
                                              : "SCHED_IDLE itself refused");
    return exitCode;
}
