#include <stdio.h>
#include <stdlib.h>

#include "periph/pm.h"
#ifdef MODULE_PERIPH_GPIO
#include "board.h"
#include "periph/gpio.h"
#endif
#ifdef MODULE_PM_LAYERED
#include "pm_layered.h"
#endif

extern int _pm_handler(int argc, char **argv);

#include "shell.h"
#include "ztimer.h"

#ifndef BTN0_INT_FLANK
#define BTN0_INT_FLANK  GPIO_RISING
#endif

#if defined(MODULE_PERIPH_GPIO_IRQ) && defined(BTN0_PIN)
static void btn_cb(void *ctx)
{
    (void) ctx;
    puts("BTN0 pressed!");
}
#endif

int main(void)
{

    puts("Pulga Low Power!");
    #ifdef MODULE_ZTIMER
    puts("ZTIMER definido!");
    #endif
    printf("You are running RIOT on a(n) %s board.\n", RIOT_BOARD);
    printf("This board features a(n) %s MCU.\n", RIOT_MCU);

    printf("This application allows you to test the CPU power management.\n"
           "The available power modes are 0 - %d. Lower-numbered power modes\n"
           "save more power, but may require an event/interrupt to wake up\n"
           "the CPU. Reset the CPU if needed.\n\n\n",
           PM_NUM_MODES - 1);

#if defined(MODULE_PERIPH_GPIO_IRQ) && defined(BTN0_PIN)
    puts("using BTN0 as wake-up source");
    gpio_init_int(BTN0_PIN, BTN0_MODE, BTN0_INT_FLANK, btn_cb, NULL);
#endif

    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(NULL, line_buf, SHELL_DEFAULT_BUFSIZE);

    return 0;
}
