// CARLA, Copyright (C) 2017 Computer Vision Center (CVC)

#pragma once

#include <carla/thread/ThreadSafeQueue.h>
#include <carla/thread/ThreadUniquePointer.h>

#include <atomic>
#include <functional>

namespace carla {
namespace thread {

  /// Executes the given job asynchronously for each item added to the queue.
  ///
  /// The job gets called each time an item is added to the queue, the item is
  /// passed as argument.
  template <typename T>
  class CARLA_API AsyncReaderJobQueue {
  public:

    using Job = std::function<void(T)>;

    explicit AsyncReaderJobQueue(Job &&job) :
        _done(false),
        _job(std::move(job)),
        _queue(),
        _thread(new std::thread(&AsyncReaderJobQueue::workerThread, this)) {}

    ~AsyncReaderJobQueue() {
      _done = true;
    }

    void push(T item) {
      _queue.push(item);
    }

  private:

    void workerThread() {
      while (!_done) {
        T value;
        _queue.wait_and_pop(value);
        _job(value);
      }
    }

    std::atomic_bool _done;

    Job _job;

    ThreadSafeQueue<T> _queue;

    const ThreadUniquePointer _thread;
  };

} // namespace thread
} // namespace carla