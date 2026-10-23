#include "base/Telemetry.h"
#include "base/detail/BackgroundPriority.h"
#include <condition_variable>
#include <iostream>
#include <map>
#include <mutex>
#include <thread>
namespace base::telemetry
{
namespace
{
class Dispatcher
{
  public:
    Dispatcher() : worker_([this] { Run(); })
    {
    }
    ~Dispatcher()
    {
        Stop();
    }
    void Post(std::string key, std::function<void()> job)
    {
        std::scoped_lock lock(mutex_);
        if (stopping_ || (!pending_.contains(key) && pending_.size() >= 256))
        {
            ++stats_.rejected;
            return;
        }
        if (pending_.contains(key))
            ++stats_.coalesced;
        pending_[std::move(key)] = std::move(job);
        wake_.notify_one();
    }
    Stats Statistics()
    {
        std::scoped_lock lock(mutex_);
        return stats_;
    }
    void Flush()
    {
        std::unique_lock lock(mutex_);
        drained_.wait(lock, [&] { return pending_.empty() && !publishing_; });
    }
    void Stop()
    {
        {
            std::scoped_lock lock(mutex_);
            stopping_ = true;
            wake_.notify_one();
        }
        if (worker_.joinable())
            worker_.join();
    }

  private:
    void Run()
    {
        if (!detail::BackgroundPriority("NT-Publish"))
            std::cerr << "[Telemetry] Could not lower publisher thread priority\n";
        std::unique_lock lock(mutex_);
        for (;;)
        {
            wake_.wait(lock, [&] { return stopping_ || !pending_.empty(); });
            if (pending_.empty() && stopping_)
                break;
            auto jobs = std::move(pending_);
            pending_.clear();
            publishing_ = true;
            lock.unlock();
            for (auto &[key, job] : jobs)
            {
                try
                {
                    job();
                }
                catch (...)
                {
                    std::scoped_lock errorLock(mutex_);
                    ++stats_.failed;
                    std::cerr << "[Telemetry] Publisher failed: " << key << '\n';
                }
            }
            lock.lock();
            publishing_ = false;
            drained_.notify_all();
        }
        drained_.notify_all();
    }
    std::mutex mutex_;
    std::condition_variable wake_, drained_;
    std::map<std::string, std::function<void()>> pending_;
    Stats stats_;
    bool stopping_{}, publishing_{};
    std::thread worker_;
};
Dispatcher &Get()
{
    static Dispatcher dispatcher;
    return dispatcher;
}
} // namespace
void Post(std::string key, std::function<void()> publish)
{
    Get().Post(std::move(key), std::move(publish));
}
Stats Statistics()
{
    return Get().Statistics();
}
void Flush()
{
    Get().Flush();
}
void Shutdown()
{
    Get().Stop();
}
} // namespace base::telemetry
