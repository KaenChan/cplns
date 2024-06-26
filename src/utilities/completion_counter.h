/* © 2014 the PBNF Authors under the MIT license. See AUTHORS for the list of
 * authors.*/

/**
 * \file completion_counter.h
 *
 *
 *
 * \author Ethan Burns
 * \date 2008-10-24
 */

#if !defined(_COMPLETION_COUNTER_H_)
#define _COMPLETION_COUNTER_H_

#include <pthread.h>

#include "common.h"

class CompletionCounter {
   public:
    CompletionCounter(void);
    CompletionCounter(unsigned int max);

    void set_max(unsigned int max);
    int get_count();

    void complete(void);
    void uncomplete(void);
    void wait(void);
    void reset(void);
    bool is_complete(void);

   private:
    std::atomic<int> counter;
    std::atomic<int> max;
};

#endif /* !_COMPLETION_COUNTER_H_ */
