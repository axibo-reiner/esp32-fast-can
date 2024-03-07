#ifndef PERFORMANCE_MONITOR_H
#define PERFORMANCE_MONITOR_H

#include <time.h>
#include <stdio.h>
#include "log.h"

#define BILLION 1000000000L

typedef struct {
    struct timespec start_time;
    struct timespec previous_time;
    int frame_count;
    double max_latency;
} performance_monitor_t;

void performance_monitor_init(performance_monitor_t* pm) {
    clock_gettime(CLOCK_MONOTONIC, &(pm->start_time));
    pm->frame_count = 0;
    pm->max_latency = 0.0;
}

void performance_monitor_update(performance_monitor_t* pm) {
    struct timespec current_time;

    //calculate total elapsed time
    clock_gettime(CLOCK_MONOTONIC, &current_time);
    double elapsed_time = (current_time.tv_sec - pm->start_time.tv_sec) + (current_time.tv_nsec - pm->start_time.tv_nsec) / (double)BILLION;  // elapsed time in seconds

    //calculate the latency
    double latency = (double)(current_time.tv_sec - pm->previous_time.tv_sec)+ (current_time.tv_nsec - pm->previous_time.tv_nsec) / (double)BILLION; ;
    if (latency > pm->max_latency){
        pm->max_latency = latency;
    }

    pm->frame_count++;
    pm->previous_time = current_time;
}

void performance_monitor_print(performance_monitor_t* pm) {
    struct timespec current_time;
    clock_gettime(CLOCK_MONOTONIC, &current_time);
    double total_elapsed_time = (current_time.tv_sec - pm->start_time.tv_sec) + (current_time.tv_nsec - pm->start_time.tv_nsec) / (double)BILLION;  // total elapsed time in seconds
    if (total_elapsed_time >= 1.0) {
        double fps = pm->frame_count / total_elapsed_time;
        log_info("PM: FPS: %f, Max Latency: %f", fps, pm->max_latency);
        pm->frame_count = 0;
        pm->max_latency = 0.0;
        pm->start_time = current_time;
    }
}

#endif // PERFORMANCE_MONITOR_H