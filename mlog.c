#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "mlog.h"
#include "filter_private.h"
#include "tmv.h"

#define MAX_FILTER_LENGTH 240 // Example maximum length

typedef struct {
    double values[MAX_FILTER_LENGTH];
    int index;
    int count;
    int length;
} MedianFilter;

typedef struct {
    struct filter filter;
    MedianFilter median_filter;
    tmv_t previous_delay;
} MLogFilter;

static void median_filter_init(MedianFilter *filter, int length) {
    filter->index = 0;
    filter->count = 0;
    filter->length = length;
    memset(filter->values, 0, sizeof(filter->values));
}

static void median_filter_add(MedianFilter *filter, double value) {
    filter->values[filter->index] = value;
    filter->index = (filter->index + 1) % filter->length;
    if (filter->count < filter->length) {
        filter->count++;
    }
}

static int compare_doubles(const void *a, const void *b) {
    double diff = *(double *)a - *(double *)b;
    return (diff > 0) - (diff < 0);
}

static double median_filter_get(MedianFilter *filter) {
    double sorted[MAX_FILTER_LENGTH];
    int size = filter->count;
    memcpy(sorted, filter->values, size * sizeof(double));
    qsort(sorted, size, sizeof(double), compare_doubles);
    if (size % 2 == 0) {
        return (sorted[size / 2 - 1] + sorted[size / 2]) / 2.0;
    } else {
        return sorted[size / 2];
    }
}

static double log_filter_apply(double current_delay, double previous_delay) {
    double change = current_delay - previous_delay;
    double threshold = 50000.0;
    double max_factor = 1.0;
    double base = 10.0;

    double adjustment_factor = log(fabs(change) / threshold + 1) / log(base + 1);
    adjustment_factor = fmin(adjustment_factor, max_factor);

    return previous_delay + adjustment_factor * change;
}

static void mlog_filter_init(MLogFilter *filter, int length) {
    median_filter_init(&filter->median_filter, length);
    filter->previous_delay = tmv_zero();
}

static void mlog_filter_reset(struct filter *filter) {
    MLogFilter *s = container_of(filter, MLogFilter, filter);
    mlog_filter_init(s, s->median_filter.length);
}

static tmv_t mlog_filter_sample(struct filter *filter, tmv_t sample) {
    MLogFilter *s = container_of(filter, MLogFilter, filter);
    median_filter_add(&s->median_filter, tmv_dbl(sample));
    double median_delay = median_filter_get(&s->median_filter);
    double filtered_delay = log_filter_apply(median_delay, tmv_dbl(s->previous_delay));
    s->previous_delay = dbl_tmv(filtered_delay);
    return s->previous_delay;
}

static void mlog_filter_destroy(struct filter *filter) {
    MLogFilter *s = container_of(filter, MLogFilter, filter);
    free(s);
}

struct filter *mlog_create(int length) {
    MLogFilter *f = calloc(1, sizeof(MLogFilter));
    if (!f)
        return NULL;

    mlog_filter_init(f, length);
    f->filter.sample = mlog_filter_sample;
    f->filter.destroy = mlog_filter_destroy;
    f->filter.reset = mlog_filter_reset;
    return &f->filter;
}

