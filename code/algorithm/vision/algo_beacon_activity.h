#ifndef ALGO_BEACON_ACTIVITY_H_
#define ALGO_BEACON_ACTIVITY_H_

#include "algo_beacon.h"

void algo_beacon_activity_reset(void);
void algo_beacon_activity_update(const uint8_t *gray_image,
                                 const algo_beacon_result_list_t *measurements,
                                 uint8_t full_detect);
uint8_t algo_beacon_activity_is_confirmed(const algo_beacon_result_t *result);
void algo_beacon_activity_filter_list(algo_beacon_result_list_t *list);

#endif
