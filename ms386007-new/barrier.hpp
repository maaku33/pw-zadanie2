#ifndef __BARRIER_HPP__
#define __BARRIER_HPP__

#include <iostream>
#include <mutex>
#include <condition_variable>

class Barrier {
    using lock_t = std::unique_lock<std::mutex>;

    std::mutex m;
    std::condition_variable cv;
    unsigned int waiting;
    bool locked;

public:
    Barrier() : waiting(0), locked(true) {}
    
    void wait() {
        lock_t lock(m);

        ++waiting;

        std::cerr << "waiting: " << waiting << std::endl;

        cv.wait(lock, [this]{ return !this->locked; });
        
        --waiting;
        if (waiting == 0) {
            locked = true;
        }
    }

    void await_for(unsigned int count) {
        lock_t lock(m);

        cv.wait(lock, [this, count]{ return this->waiting >= count; });
    }

    void open() {
        lock_t lock(m);
        locked = false;
        cv.notify_all();
    }
};

#endif /* __BARRIER_HPP__ */