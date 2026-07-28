// phys2d — пул потоков для узкой фазы + безблокировочный список контактов.
#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>
#include <cstddef>

#ifdef PHYS2D_USE_TBB
#  include <tbb/parallel_for.h>
#  include <tbb/blocked_range.h>
#endif

namespace phys2d {

// Пул потоков с parallelFor и барьером ожидания.
class ThreadPool {
public:
    explicit ThreadPool(int threads = 0) {
        int n = (threads > 0) ? threads : (int)std::thread::hardware_concurrency();
        if (n < 1) n = 1;
        m_threadCount = n;
        for (int i = 1; i < n; ++i)
            m_workers.emplace_back([this] { workerLoop(); });
    }
    ~ThreadPool() {
        {
            std::lock_guard<std::mutex> lk(m_mutex);
            m_stop = true;
        }
        m_cv.notify_all();
        for (auto& t : m_workers) if (t.joinable()) t.join();
    }

    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

    int threadCount() const { return m_threadCount; }

    // Параллельный проход по блокам [0..count).
    void parallelFor(size_t count, size_t grain, const std::function<void(size_t, size_t)>& fn) {
        if (count == 0) return;
        if (m_threadCount <= 1 || count <= grain) { fn(0, count); return; }
#ifdef PHYS2D_USE_TBB
        tbb::parallel_for(tbb::blocked_range<size_t>(0, count, grain),
                          [&](const tbb::blocked_range<size_t>& r) { fn(r.begin(), r.end()); });
#else
        const size_t chunk = std::max(grain, (count + (size_t)m_threadCount - 1) / (size_t)m_threadCount);
        const size_t jobs  = (count + chunk - 1) / chunk;
        m_pending.store((int)jobs, std::memory_order_release);
        {
            std::lock_guard<std::mutex> lk(m_mutex);
            m_jobs.clear();
            for (size_t j = 0; j < jobs; ++j) {
                const size_t b = j * chunk;
                const size_t e = std::min(count, b + chunk);
                m_jobs.push_back([&fn, b, e] { fn(b, e); });
            }
            m_next.store(0, std::memory_order_release);
        }
        m_cv.notify_all();
        runJobs();                                   // главный поток тоже работает
        while (m_pending.load(std::memory_order_acquire) > 0) std::this_thread::yield();
#endif
    }

private:
    void runJobs() {
        for (;;) {
            const size_t i = m_next.fetch_add(1, std::memory_order_acq_rel);
            std::function<void()> job;
            {
                std::lock_guard<std::mutex> lk(m_mutex);
                if (i >= m_jobs.size()) return;
                job = m_jobs[i];
            }
            job();
            m_pending.fetch_sub(1, std::memory_order_acq_rel);
        }
    }
    void workerLoop() {
        for (;;) {
            {
                std::unique_lock<std::mutex> lk(m_mutex);
                m_cv.wait(lk, [this] { return m_stop || m_next.load() < m_jobs.size(); });
                if (m_stop) return;
            }
            runJobs();
        }
    }

    std::vector<std::thread>           m_workers;
    std::vector<std::function<void()>> m_jobs;
    std::mutex                         m_mutex;
    std::condition_variable            m_cv;
    std::atomic<size_t>                m_next{0};
    std::atomic<int>                   m_pending{0};
    bool                               m_stop = false;
    int                                m_threadCount = 1;
};

// Безблокировочный список: предвыделенный буфер + атомарный индекс записи.
// Потоки узкой фазы пишут контакты без мьютексов.
template <typename T>
class LockFreeList {
public:
    void reserve(size_t capacity) { m_data.resize(capacity); m_size.store(0, std::memory_order_relaxed); }
    void clear() { m_size.store(0, std::memory_order_relaxed); }

    // Возвращает false если буфер переполнен (тогда контакт отбрасывается).
    bool push(const T& v) {
        const size_t i = m_size.fetch_add(1, std::memory_order_acq_rel);
        if (i >= m_data.size()) { m_size.store(m_data.size(), std::memory_order_release); return false; }
        m_data[i] = v;
        return true;
    }
    size_t size() const { return std::min(m_size.load(std::memory_order_acquire), m_data.size()); }
    size_t capacity() const { return m_data.size(); }
    T*       data()       { return m_data.data(); }
    const T* data() const { return m_data.data(); }
    T&       operator[](size_t i)       { return m_data[i]; }
    const T& operator[](size_t i) const { return m_data[i]; }
    size_t bytes() const { return m_data.capacity() * sizeof(T); }

private:
    std::vector<T>      m_data;
    std::atomic<size_t> m_size{0};
};

} // namespace phys2d
