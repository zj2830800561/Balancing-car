#ifndef __MESSAGE_H_
#define __MESSAGE_H_

#include <rtthread.h>

void Print_thread_entry(void);
void AngleSend(rt_uint16_t AcclAngle, rt_uint16_t GyroAngle, rt_uint16_t FilterAngle);
void SpeedSend(int target, int Lcurrent, int Rcurrent);

#endif /* __MESSAGE_H_ */
