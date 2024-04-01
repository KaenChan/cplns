/*
 * buffer.hpp
 *
 *  Created on: Jul 21, 2014
 *      Author: yuu
 */

#ifndef BUFFER_HPP_
#define BUFFER_HPP_

#include <vector>
#include <pthread.h>

template<typename T>
class hd_msg_buffer {
private:
	std::vector<T*> buf;
	pthread_mutex_t m;

public:
	hd_msg_buffer() {
		pthread_mutex_init(&m, NULL);
	}

	// Push without sync
	void push_with_lock(T* x) {
		buf.push_back(x);
	}

	// Push without sync
	void push_all_with_lock(std::vector<T*> buffer) {
		buf.insert(buf.end(), buffer.begin(), buffer.end());
	}

	void push(T* x) {
		pthread_mutex_lock(&m);
		buf.push_back(x);
//		printf("pushed %d\n",buf.back().num);
		pthread_mutex_unlock(&m);
	};

	bool try_push(T* x) {
		// trylock return 0 if it acquires the lock.
		if (pthread_mutex_trylock(&m)) {
			return false;
		}
		buf.push_back(x);
		pthread_mutex_unlock(&m);
		return true;
	}

	// Returns true if it acquires the lock.
	bool try_lock() {
		// mutex trylock returns 0 if it acquires lock.
		return !pthread_mutex_trylock(&m);
	}

	void lock() {
		pthread_mutex_lock(&m);
	}

	void release_lock() {
		pthread_mutex_unlock(&m);
	}

	T* pull() {
		pthread_mutex_lock(&m);
		T* ret = buf.back();
		buf.pop_back();
//		printf("pulled %d\n", ret.num);
		pthread_mutex_unlock(&m);
		return ret;
	};

	void pull_all(T* ret[]) {
		pthread_mutex_lock(&m);
		std::copy(buf.begin(), buf.end(), ret);
		buf.clear();
		pthread_mutex_unlock(&m);
	};

	// TODO: Might be better to use move (C++11)
	std::vector<T*> pull_all() {
		pthread_mutex_lock(&m);
//		std::copy(buf.begin(), buf.end(), ret);
		std::vector<T*> ret(buf);
		buf.clear();
		pthread_mutex_unlock(&m);
		return ret;
	};

	std::vector<T*> pull_all_with_lock() {
		std::vector<T*> ret(buf);
		buf.clear();
		return ret;
	};

	bool isempty(){
		return buf.empty();
		// bool b;
		// pthread_mutex_lock(&m);
		// b = buf.empty();
		// pthread_mutex_unlock(&m);
		// return b;
	}

	int size() {
		return buf.size();
		// int b;
		// pthread_mutex_lock(&m);
		// b = buf.size();
		// pthread_mutex_unlock(&m);
		// return b;
	}

};

#endif /* BUFFER_HPP_ */
