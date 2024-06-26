// Copyright (c) 2015 Tomas Balyo, Karlsruhe Institute of Technology
/*
 * Threading.h
 *
 *  Created on: Nov 25, 2014
 *      Author: balyo
 */

#ifndef THREADING_H_
#define THREADING_H_

#include <atomic>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>

#define TESTRUN(cmd, msg) int res = cmd; if (res != 0) { printf(msg,res); exit(res); }

class lock_base {
public:
	virtual void lock() = 0;
	virtual void unlock() = 0;
	virtual bool tryLock() = 0;
};

class Mutex : public lock_base {
private:
	pthread_mutex_t mtx;
public:
	Mutex() {
		TESTRUN(pthread_mutex_init(&mtx, NULL), "Mutex init failed with msg %d\n")
	}
	virtual ~Mutex() {
		TESTRUN(pthread_mutex_destroy(&mtx), "Mutex destroy failed with msg %d\n")
	}
	void lock() {
		TESTRUN(pthread_mutex_lock(&mtx), "Mutex lock failed with msg %d\n")
	}
	void unlock() {
		TESTRUN(pthread_mutex_unlock(&mtx), "Mutex unlock failed with msg %d\n")
	}
	bool tryLock() {
		// return true if lock acquired
		return pthread_mutex_trylock(&mtx) == 0;
	}
};

class Thread {
private:
	pthread_t thread;
public:
	Thread(void*(*method)(void*), void* arg) {
		pthread_create(&thread, NULL, method, arg);
	}
	void join() {
		pthread_join(thread, NULL);
	}
};

class spinlock : public lock_base {
private:
  std::atomic<bool> lock_ = {false};

public:
  spinlock() { lock_ = false; }

  void lock() noexcept {
    for (;;) {
      // Optimistically assume the lock is free on the first try
      if (!lock_.exchange(true, std::memory_order_acquire)) {
        return;
      }
      // Wait for lock to be released without generating cache misses
      while (lock_.load(std::memory_order_relaxed)) {
        // Issue X86 PAUSE or ARM YIELD instruction to reduce contention between
        // hyper-threads
        __builtin_ia32_pause();
      }
    }
  }

  bool tryLock() noexcept {
    // First do a relaxed load to check if lock is free in order to prevent
    // unnecessary cache misses if someone does while(!try_lock())
    return !lock_.load(std::memory_order_relaxed) &&
           !lock_.exchange(true, std::memory_order_acquire);
  }

  void unlock() noexcept {
    lock_.store(false, std::memory_order_release);
  }
};


#endif /* THREADING_H_ */
