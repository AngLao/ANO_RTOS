#ifndef _DRV_TIMER_H_
#define _DRV_TIMER_H_

#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()  vConfigureTimerForRunTimeStats()
#define portALT_GET_RUN_TIME_COUNTER_VALUE  timer_count


void vConfigureTimerForRunTimeStats(void);
 

#endif

