#include "stdint.h"
#include "string.h"

#include "board.h"
#include "debugpins.h"
#include "opentimers.h"
#include "board_info.h"
#include "xtimer.h"

#include "ps.h"

//=========================== defines =========================================

//#define SCTIMER_PERIOD     (32768) // @32kHz = 1s
//#define SCTIMER_PERIODIC_TICKS     (50) // @32kHz = 1s
#define SCTIMER_PERIOD     (7) // @32kHz = 1s
#define SCTIMER_MIN        (1000) // @32kHz = 1s
#define SCTIMER_MAX        (2000) // @32kHz = 1s
#define SCTIMER_REPEAT     (30) // @32kHz = 1s

#define SCTIMER_PERIODIC_MIN        (300) // @32kHz = 1s
#define SCTIMER_PERIODIC_MAX        (500) // @32kHz = 1s

//=========================== prototypes ======================================

void cb_compare(void);

#define SCHEDULE_DURATION (10000)

opentimers_id_t timer_id0;
opentimers_id_t timer_id1;
opentimers_id_t timer_id2;

//=========================== main ============================================
int period = SCTIMER_MIN;
int cnt = 0;
int one_shot_increment = 1;
 void opentimer_cb(opentimers_id_t id)
 {

     printf("o");
     cnt++;
     if (cnt >= SCTIMER_REPEAT) {
         printf("\n");
         cnt = 0;
         period += one_shot_increment;
        // if (id == timer_id0) {
        //     period += 2;
        // }
        // else if (id == timer_id1) {
        //     period -= 2;
        // }
         if ((period <= SCTIMER_MIN) || (period >= SCTIMER_MAX)) {
             //period = SCTIMER_MAX;
             one_shot_increment *= -1;
         }
         printf("%lu: opentimer_cb (%d) will do %d one-shot callbacks, one every %lu ticks...\n",
                xtimer_now_usec(), id, SCTIMER_REPEAT, period);
     }

     opentimers_scheduleAbsolute(
      id,                  // timerId
      period,       // duration
      sctimer_readCounter(),     // reference
      TIME_TICS,                 // timetype
      opentimer_cb               // callback
     );
 }

int periodic_cnt = 0;
timer_type_t timer_type = TIMER_PERIODIC; //TIMER_ONESHOT
time_type_t time_type = TIME_TICS;
int periodic_duration = SCTIMER_PERIODIC_MAX;
int periodic_increment = 2;
//time_type_t time_type = TIME_MS;
void periodic_cb(opentimers_id_t id)
{
     periodic_cnt++;
     printf("p");
     if (periodic_cnt >= SCTIMER_REPEAT) {
         printf("\n");
         periodic_cnt = 0;
         opentimers_cancel(id);
         if ((periodic_duration <= SCTIMER_PERIODIC_MIN) || (periodic_duration >= SCTIMER_PERIODIC_MAX)) {
             //period = SCTIMER_MAX;
             periodic_increment *= -1;
         }
         periodic_duration += periodic_increment;

         printf("%lu: periodic_cb (%d) scheduling %ld periodic callbacks each %ld ticks\n",
                xtimer_now_usec(), id, SCTIMER_REPEAT, periodic_duration);
         opentimers_scheduleIn(id, periodic_duration, time_type, timer_type,
                               periodic_cb);
     }
}

/**
\brief The program starts executing here.
*/
int main(void)
{
   xtimer_init();
   xtimer_sleep(1);

   sctimer_init();
   xtimer_usleep(1000);
   printf("sctimer_readCounter(): %lu\n", sctimer_readCounter());
   opentimers_init();
   timer_id0 = opentimers_create();
   timer_id1 = opentimers_create();
   //timer_id1 = opentimers_create();
   //timer_id2 = opentimers_create();
   //time_type_t time_type = TIME_TICS;
   //time_type_t time_type = TIME_MS;

   //opentimers_setPriority(timer_id0, 1);
   //opentimers_setPriority(timer_id1, 1);

   opentimer_cb(timer_id0);

   //for (int i = 0; i < 100; i++) {
   //    printf("%lu\n", sctimer_readCounter());
   //    xtimer_sleep(1);
   //}

   //xtimer_usleep(200000);
   //opentimer_cb(timer_id1);
   //xtimer_usleep(100000);
   //opentimer_cb(timer_id2);
   //timer_type_t timer_type = TIMER_PERIODIC; //TIMER_ONESHOT
   //opentimers_scheduleIn(timer_id, SCHEDULE_DURATION, time_type, timer_type,
   //                      opentimer_cb);
   opentimers_scheduleIn(timer_id1, periodic_duration, time_type, timer_type,
                         periodic_cb);
}
