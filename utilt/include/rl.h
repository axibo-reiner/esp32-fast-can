/**
 * @file rl.h   
 * @author Reiner Schmidt 
 * @brief 
 * @version 0.1
 * @date 2024-03-01
 * 
 * @copyright Copyright Axibo Inc(c) 2024
 * 
 */
#include <time.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include "log.h"

#define BILLION 1000000000L
#define NSEC_PER_SEC 1000000000

void usleep_n(unsigned int usec);


typedef struct {
    struct timespec start_time;
    int counter;
    double time_per_iteration;
} rate_limiter;

void rate_limiter_init(rate_limiter* rl, int rate) {
    clock_gettime(CLOCK_MONOTONIC, &(rl->start_time));
    rl->counter = 1;
    rl->time_per_iteration = 1.0 / (float)rate;  // time per iteration in seconds
}

void rate_limiter_limit(rate_limiter* rl) {
    struct timespec current_time;
    struct timespec sleep_time = {0, 0};  // Initialize sleep_time
    clock_gettime(CLOCK_MONOTONIC, &current_time);
    double elapsed_time_sec = (double)(current_time.tv_sec - rl->start_time.tv_sec) + (double)(current_time.tv_nsec - rl->start_time.tv_nsec) / (double)BILLION;  // elapsed time in seconds
    double sleep_time_sec = ((rl->time_per_iteration * (double)(rl->counter)) - elapsed_time_sec);

    if (sleep_time_sec < 0){
        sleep_time_sec = 0.00001;
    }

    rl->counter++;
    usleep_n(sleep_time_sec * 1.0e6);
}

long rate_limiter_get_required_limit(rate_limiter* rl) {
    struct timespec current_time;
    struct timespec sleep_time = {0, 0};  // Initialize sleep_time
    clock_gettime(CLOCK_MONOTONIC, &current_time);
    double elapsed_time_sec = (double)(current_time.tv_sec - rl->start_time.tv_sec) + (double)(current_time.tv_nsec - rl->start_time.tv_nsec) / (double)BILLION;  // elapsed time in seconds
    double sleep_time_sec = ((rl->time_per_iteration * (double)(rl->counter)) - elapsed_time_sec);

    if (sleep_time_sec < 0){
        sleep_time_sec = 0.00001;
    }

    rl->counter++;
    return  sleep_time_sec * 1.0e6;
}

void usleep_n(unsigned int usec) {
    struct timeval start, current;
    unsigned int elapsed;

    // Get the start time
    gettimeofday(&start, NULL);

    // Loop until the elapsed time is greater than or equal to the sleep time
    while (1) {
        // Get the current time
        gettimeofday(&current, NULL);

        // Calculate the elapsed time in microseconds
        elapsed = (current.tv_sec - start.tv_sec) * 1000000 + (current.tv_usec - start.tv_usec);

        // If the elapsed time is greater than or equal to the sleep time, break out of the loop
        if (elapsed >= usec) {
            break;
        }

        // Yield the scheduler
        sched_yield();
    }
}