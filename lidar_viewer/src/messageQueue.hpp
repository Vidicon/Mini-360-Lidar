#include <iostream>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>

template<typename T>
class MessageQueue {
public:
    // Add a message to the queue
    void push(T message) {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.push(message);
        condVar_.notify_one(); // Notify one waiting thread
    }

    // Retrieve and remove a message from the queue
    T pop() {
        std::unique_lock<std::mutex> lock(mutex_);
        condVar_.wait(lock, [this] { return !queue_.empty(); }); // Wait until the queue is not empty
        T message = queue_.front();
        queue_.pop();
        return message;
    }

    bool empty() {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }

private:
    std::queue<T> queue_;
    std::mutex mutex_;
    std::condition_variable condVar_;
};
