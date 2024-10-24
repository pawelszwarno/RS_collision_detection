#ifndef __CPS_COLLISION_DETECTION__
#define __CPS_COLLISION_DETECTION__

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus
#include <unistd.h>


typedef void (*event_cb(void*))

int collision_detection_init(uint8_t min_dist, event_cb collision_cb);
int detect_collision(void);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // __CPS_COLLISION_DETECTION__