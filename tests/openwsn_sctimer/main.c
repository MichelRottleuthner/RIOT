#include "stdint.h"
#include "string.h"

#include "board.h"
#include "debugpins.h"
#include "sctimer.h"

#include "ps.h"

//=========================== defines =========================================

#define SCTIMER_PERIOD     (32768) // @32kHz = 1s

//=========================== prototypes ======================================

void cb_compare(void);

//=========================== main ============================================

/**
\brief The program starts executing here.
*/
int main(void)
{
   sctimer_init();
   sctimer_set_callback(cb_compare);
   /* bootstrapping repeating callback by calling it manually once */
   cb_compare();
}

//=========================== callbacks =======================================
int counter=0;
void cb_compare(void) {
   uint32_t now = sctimer_readCounter();
   uint32_t next_wakeup = now + SCTIMER_PERIOD;
   sctimer_setCompare(next_wakeup);

   printf("%i. cb_compare, sctimer: %" PRIu32 ", next wakeup: %" PRIu32 "\n",
          counter++, now, next_wakeup);

   LED0_TOGGLE;
   sctimer_setCompare(next_wakeup);
}
