#ifndef THREADSAFEQUEUE_H
#define THREADSAFEQUEUE_H

#include <iostream>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <cstdint>

// 通用的消息结构体
struct GenericMessage
{
    int type;   // 消息类型
    void *data; // 指向数据的指针
    // size_t size;        // 数据的大小

    GenericMessage(int t, void *d) : type(t), data(d) {}
};

// 线程安全的消息队列
class ThreadSafeQueue
{
private:
    std::queue<GenericMessage> queue_;
    mutable std::mutex mutex_;
    std::condition_variable cond_var_;
    bool stop_;
    std::function<void(ThreadSafeQueue &, GenericMessage)> processFunction_;

public:
    ThreadSafeQueue();

    // 入队操作
    void push(const GenericMessage &msg);

    // 出队操作，如果队列为空则阻塞等待
    GenericMessage pop();

    // 检查队列是否为空
    bool empty() const;

    // 检查队列是否停止接收
    bool isStop() const;

    // 停止接收等待
    void stopQueue();

    // 启动后台处理线程
    void startQueue(std::function<void(ThreadSafeQueue &, GenericMessage)> processFunc);
};

#endif // THREADSAFEQUEUE_H
