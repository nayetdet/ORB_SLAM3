// How long can a SCHED_IDLE thread stay descheduled while it holds a lock another thread needs?
//
// Model of the dense thread (Dense.lowPriority, SCHED_IDLE) when PrioBoost cannot raise it back to
// SCHED_OTHER (see prio_boost_probe.cc): every `period` ms it wakes from a timed condition-variable
// wait, locks a mutex, does `hold` microseconds of CPU work inside it and unlocks -- what Run() does
// with mMutexQueue (wait_for + pop_front). A "tracking" thread at normal priority takes the same
// mutex every `waiter` ms, as Enqueue() does for every keyframe. `hogs` busy threads at normal
// priority keep the CPUs saturated, as ORB-SLAM3's own threads do on a small cpuset.
//
// Reported per run: the holder's wall-clock time between lock() and unlock() (the time any waiter
// arriving in that window is blocked) and the waiter's own measured wait for the lock.
//
// Build:  g++ -O2 -std=c++17 -pthread idle_holder_stall.cc -o idle_holder_stall
// Run:    ./idle_holder_stall <seconds> <hogs> <hold_us> <idle|other> [period_ms=50] [waiter_ms=33]
//         docker run --cpuset-cpus 0,6,1,7 ... ./idle_holder_stall 60 4 50 idle

#include <pthread.h>
#include <sched.h>
#include <time.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using clk = std::chrono::steady_clock;

static double cpuNow()   // CPU time of the calling thread, seconds
{
    timespec ts;
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
    return static_cast<double>(ts.tv_sec) + 1e-9 * static_cast<double>(ts.tv_nsec);
}

static void report(const char *name, std::vector<double> v)   // v in milliseconds
{
    if (v.empty()) { std::printf("  %-26s no samples\n", name); return; }
    std::sort(v.begin(), v.end());
    auto q = [&](double p) { return v[std::min(v.size() - 1, static_cast<size_t>(p * static_cast<double>(v.size())))]; };
    size_t g1 = 0, g10 = 0, g100 = 0, g1000 = 0;
    for (double x : v) { g1 += x > 1; g10 += x > 10; g100 += x > 100; g1000 += x > 1000; }
    std::printf("  %-26s n=%zu  p50 %.3f  p99 %.3f  p99.9 %.3f  max %.3f ms   >1ms %zu  >10ms %zu  >100ms %zu  >1s %zu\n",
                name, v.size(), q(0.5), q(0.99), q(0.999), v.back(), g1, g10, g100, g1000);
}

int main(int argc, char **argv)
{
    if (argc < 5) { std::fprintf(stderr, "usage: %s seconds hogs hold_us idle|other [period_ms] [waiter_ms]\n", argv[0]); return 2; }
    const double seconds = std::atof(argv[1]);
    const int hogs = std::atoi(argv[2]);
    const double holdUs = std::atof(argv[3]);
    const bool idle = std::string(argv[4]) == "idle";
    const int periodMs = argc > 5 ? std::atoi(argv[5]) : 50;
    const int waiterMs = argc > 6 ? std::atoi(argv[6]) : 33;

    std::mutex m;
    std::atomic<bool> stop{false};
    std::vector<std::thread> pool;
    for (int i = 0; i < hogs; ++i)
        pool.emplace_back([&] { volatile double x = 1.0; while (!stop) x = x * 1.0000001 + 1e-9; });

    std::vector<double> holdWall, waiterWait;
    std::string holderPolicy = "?";
    std::thread holder([&] {
        sched_param sp; sp.sched_priority = 0;
        if (idle && pthread_setschedparam(pthread_self(), SCHED_IDLE, &sp) != 0) { std::perror("SCHED_IDLE"); std::exit(3); }
        holderPolicy = sched_getscheduler(0) == SCHED_IDLE ? "SCHED_IDLE" : "SCHED_OTHER";
        std::mutex cvm;
        std::condition_variable cv;
        while (!stop)
        {
            {   // Run(): mQueueUpdated.wait_for(lock, 50 ms, pred) on its own condition variable
                std::unique_lock<std::mutex> l(cvm);
                cv.wait_for(l, std::chrono::milliseconds(periodMs), [&] { return stop.load(); });
            }
            std::unique_lock<std::mutex> lock(m);
            const auto t0 = clk::now();
            const double c0 = cpuNow();
            while ((cpuNow() - c0) * 1e6 < holdUs) {}
            const double wall = std::chrono::duration<double, std::milli>(clk::now() - t0).count();
            lock.unlock();
            holdWall.push_back(wall);
        }
    });
    std::thread waiter([&] {
        while (!stop)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(waiterMs));
            const auto t0 = clk::now();
            std::unique_lock<std::mutex> lock(m);
            const double wait = std::chrono::duration<double, std::milli>(clk::now() - t0).count();
            lock.unlock();
            waiterWait.push_back(wait);
        }
    });

    std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
    stop = true;
    holder.join(); waiter.join();
    for (auto &t : pool) t.join();

    std::printf("holder %s, %d hogs, hold %.0f us of CPU, holder period %d ms, waiter period %d ms, %.0f s\n",
                holderPolicy.c_str(), hogs, holdUs, periodMs, waiterMs, seconds);
    report("holder: wall lock..unlock", holdWall);
    report("waiter: wait for lock", waiterWait);
    return 0;
}
