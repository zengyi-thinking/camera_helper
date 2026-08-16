#ifndef ALGO_ONELAMP_H_
#define ALGO_ONELAMP_H_

#include "algo_vision_target.h"

#ifdef __cplusplus
extern "C" {
#endif

void algo_onelamp_reset(void);
void algo_onelamp_process(const uint8 *src_image);
const algo_vision_target_result_t *algo_onelamp_get_result(void);

#ifdef __cplusplus
}
#endif

#endif
