#include "periph_cpu.h"
#include "cpu.h"
#include "pm_layered.h"

#include "clk.h"
#include "board.h"
#include "periph_conf.h"
#include "timex.h"
#include "ztimer.h"
#include "nrf_clock.h"
#include "ztimer/periph_rtc.h"

#define RTC_FREQ 32768
#define RTC_INTERVAL 10

void nrf52_sys_on(void) {
    /* Clear Event Register */
    __SEV();
    /* Wait for event */
    __WFE();
    /* Wait for event */
    __WFE();
}

void RTC1_IRQHandler(void)
{
    if(NRF_RTC1->EVENTS_COMPARE[0] == 1) {  
        NRF_RTC1->EVENTS_COMPARE[0] = 0;
    } 
}

// Set the RTC to trigger a wake-up interrupt every 10 second
void clock_config(void) {
    // Configure the RTC to use the low-frequency crystal
    NRF_CLOCK->LFCLKSRC = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_LFCLKSTART = 1;

    // Wait for the low-frequency crystal to start
    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0);

}
void rtc_config(void)
{ 

    NRF_RTC1->PRESCALER = 0;
    NRF_RTC1->CC[0] = RTC_INTERVAL * RTC_FREQ;

    NRF_RTC1->INTENSET = RTC_INTENSET_COMPARE0_Msk; // Enable interrupt on compare event 0
    NRF_RTC1->TASKS_CLEAR = 1;
    NRF_RTC1->TASKS_START = 1;

}


void pm_set(unsigned mode)
{
    clock_config();
    switch (mode) {
        case 0:
            //RTC wake up source sleep
            // Initialize the RTC

            rtc_config();
            puts("Entering sleep");
            nrf52_sys_on();
            RTC1_IRQHandler();

            puts("Exiting sleep");
            pm_set(1);

        break;
        case 1:
            //Ztimer active mode
            puts("Entering active");
     
            ztimer_sleep(ZTIMER_MSEC, 10 * MS_PER_SEC);

            pm_set(0);

        break;
        case 2:
            //No sleep
            
        break;
    }
}

void pm_off(void);

void pm_set_lowest(void);