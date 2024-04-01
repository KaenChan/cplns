// © 2014 the PBNF Authors under the MIT license. See AUTHORS for the list of
// authors.

/**
 * \file completion_counter.cc
 *
 *
 *
 * \author Ethan Burns
 * \date 2008-10-24
 */

#include "completion_counter.h"

#include <pthread.h>

#include <iostream>

using namespace std;

CompletionCounter::CompletionCounter(unsigned int max) : counter(0), max(max) {}

CompletionCounter::CompletionCounter(void) : counter(0), max(0) {}

/**
 * Set the maximum value.
 */
void CompletionCounter::set_max(unsigned int m) { this->max = m; }

int CompletionCounter::get_count() {
    int ret;
    ret = counter.load();
    return ret;
}

bool CompletionCounter::is_complete() {
    bool ret;
    ret = counter.load() >= max.load();
    return ret;
}

void CompletionCounter::complete(void) { counter++; }

void CompletionCounter::uncomplete(void) { counter--; }

/**
 * Wait for all the completions. (counter == max)
 */
void CompletionCounter::wait(void) {
    while (counter.load() < max.load()) {
    }
}

/**
 * Reset the counter to zero.
 */
void CompletionCounter::reset(void) { counter = 0; }
