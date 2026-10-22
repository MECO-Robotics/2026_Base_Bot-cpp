#include "base/FrameRecorder.h"
#include "base/detail/BackgroundPriority.h"
#include <algorithm>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>
namespace base {
class FrameRecorder::Impl {
 public:
  Impl(Sink sink,FaultSink fault,size_t capacity)
      :sink_(std::move(sink)),fault_(std::move(fault)),queue_(capacity) {
    if(!capacity||!sink_||!fault_)throw std::invalid_argument("Invalid frame recorder configuration");
    worker_=std::thread([this]{Run();});
  }
  ~Impl(){Stop();}
  bool Submit(wpi::json snapshot,int64_t timestamp) {
    std::scoped_lock lock(mutex_);
    if(stopping_)return false;
    if(stats_.rejected||stats_.failed||size_==queue_.size()) {
      ++stats_.rejected;wake_.notify_one();return false;
    }
    queue_[(head_+size_)%queue_.size()]={std::move(snapshot),timestamp};
    ++size_;stats_.highWater=std::max(stats_.highWater,size_);wake_.notify_one();return true;
  }
  Stats Statistics() const {std::scoped_lock lock(mutex_);return stats_;}
  void Flush() {
    std::unique_lock lock(mutex_);
    drained_.wait(lock,[&]{return size_==0&&!busy_&&!Unreported();});
  }
  void Stop() {
    {std::scoped_lock lock(mutex_);stopping_=true;wake_.notify_one();}
    if(worker_.joinable())worker_.join();
  }
 private:
  struct Item {wpi::json snapshot;int64_t timestamp{};};
  bool Unreported() const {return (stats_.rejected||stats_.failed)&&!faultReported_;}
  void Run() {
    if(!detail::BackgroundPriority("Frame-Record"))
      std::cerr<<"[Replay] Could not lower recorder thread priority\n";
    for(;;) {
      Item item;Stats fault;bool report=false,write=false;
      {
        std::unique_lock lock(mutex_);
        wake_.wait(lock,[&]{return stopping_||size_||Unreported();});
        if(Unreported()) {
          fault=stats_;faultReported_=true;report=true;
        }
        if(size_) {item=std::move(queue_[head_]);head_=(head_+1)%queue_.size();--size_;write=true;}
        if(!write&&!report&&stopping_)break;
        busy_=true;
      }
      if(report) {
        std::cerr<<"[Replay] Recording invalid: rejected="<<fault.rejected<<", failed="<<fault.failed<<'\n';
        try {fault_(fault);}catch(...) {std::cerr<<"[Replay] Could not persist recording fault\n";}
      }
      if(write) {
        try {
          sink_(item.snapshot,item.timestamp);
          std::scoped_lock lock(mutex_);++stats_.written;
        } catch(...) {std::scoped_lock lock(mutex_);++stats_.failed;}
      }
      {std::scoped_lock lock(mutex_);busy_=false;drained_.notify_all();}
    }
    drained_.notify_all();
  }
  Sink sink_;FaultSink fault_;
  std::vector<Item> queue_;
  mutable std::mutex mutex_;
  std::condition_variable wake_,drained_;
  size_t head_{},size_{};
  Stats stats_;
  bool faultReported_{},stopping_{},busy_{};
  std::thread worker_;
};
FrameRecorder::FrameRecorder(Sink sink,FaultSink fault,size_t capacity)
    :impl_(std::make_unique<Impl>(std::move(sink),std::move(fault),capacity)){}
FrameRecorder::~FrameRecorder()=default;
bool FrameRecorder::Submit(wpi::json snapshot,int64_t timestamp){return impl_->Submit(std::move(snapshot),timestamp);}
FrameRecorder::Stats FrameRecorder::Statistics() const{return impl_->Statistics();}
void FrameRecorder::Flush(){impl_->Flush();}
void FrameRecorder::Stop(){impl_->Stop();}
}
