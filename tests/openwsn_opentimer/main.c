#include "stdint.h"
#include "string.h"

#include "board.h"
#include "debugpins.h"
#include "opentimers.h"
#include "board_info.h"

#include "ps.h"

//=========================== defines =========================================

#define SCTIMER_PERIOD     (32768) // @32kHz = 1s

//=========================== prototypes ======================================

void cb_compare(void);

#define SCHEDULE_DURATION (10000)

//=========================== main ============================================

 void opentimer_cb(opentimers_id_t id)
 {
     printf("opentimer_cb\n");
 }

/**
\brief The program starts executing here.
*/
int main(void)
{
   opentimers_init();
   opentimers_id_t timer_id = opentimers_create();
   //time_type_t time_type = TIME_TICS;
   //time_type_t time_type = TIME_MS;

   opentimers_scheduleAbsolute(
    timer_id,                  // timerId
    SCHEDULE_DURATION,       // duration
    sctimer_readCounter(),     // reference
    TIME_TICS,                 // timetype
    opentimer_cb               // callback
   );

   //timer_type_t timer_type = TIMER_PERIODIC; //TIMER_ONESHOT
   //opentimers_scheduleIn(timer_id, SCHEDULE_DURATION, time_type, timer_type,
   //                      opentimer_cb);

}
