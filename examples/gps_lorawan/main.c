/*
 * Copyright (C) 2024 Universade de São Paulo
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     examples
 * @{
 *
 * @file
 * @brief       A LoraWAN example with GPS
 *
 * @author      Kauê Rodrigues Barbosa <kaue.rodrigueskrb@usp.br>
 *
 * @}
 */


#include <limits.h>
#include <float.h>

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "bq2429x.h"

#include "assert.h"
#include "event/timeout.h"

#include "msg.h"
#include "thread.h"
#include "fmt.h"

#include "net/loramac.h"
#include "semtech_loramac.h"

#include "sx127x.h"
#include "sx127x_netdev.h"
#include "sx127x_params.h"

#include "periph/gpio.h"
#include "xtimer.h"

#include <inttypes.h>

#include "board.h"
#include "periph/adc.h"

#include "ringbuffer.h"
#include "periph/uart.h"

#include "minmea.h"

#include "ztimer.h"
#include "shell.h"

#define PMTK_SET_NMEA_OUTPUT_RMC    "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"
#define PMTK_SET_UPDATE_F_2HZ       "$PMTK300,500,0,0,0,0*28\r\n"

//Constantes

#define DATE_TIME_LEN (6U)
#define LATITUDE_LEN (4U)
#define LONGITUDE_LEN (4U)
#define SPEED_LEN (3U)

//const size_t DATE_TIME_LEN = 6;
//const size_t LATITUDE_LEN = 4;
//const size_t LONGITUDE_LEN = 4;
//const size_t SPEED_LEN = 3;

// Tamanho string final = 27 (26 bytes + \n)

char HDR;
char DEVID[LORAMAC_DEVADDR_LEN];
char DEFAULT;
char DATETIME[DATE_TIME_LEN];
char LATITUDE[LATITUDE_LEN]; 
char LONGITUDE[LONGITUDE_LEN];
char SPEED[SPEED_LEN];

//char HDR = 0x10;
//char DEVID[LORAMAC_DEVADDR_LEN+1] = {0x00,0x00,0x00,0x00,'\n'};
//char DEFAULT = 0x00;
//char DATETIME[DATE_TIME_LEN+1] = {0x18,0x02,0x09,0x13,0x25,0x02,'\n'};
//char LATITUDE[LATITUDE_LEN+1] = {0x16,0x94,0x95,0x75,'\n'}; 
//char LONGITUDE[LONGITUDE_LEN+1] = {0x2b,0x19,0x18,0x00,'n'};
//char SPEED[SPEED_LEN+1] = {0x00,0x00,0x00,'\n'};

//char p_datetime[8];
//p_datetime = (char *)&DATETIME;


int minmea_getdatetime(struct tm *tm, const struct minmea_date *date, const struct minmea_time *time_);


// Era pra ser importado do pkg. Ver isso depois
#ifndef MINMEA_MAX_SENTENCE_LENGTH
#define MINMEA_MAX_SENTENCE_LENGTH 80
#endif

/**
*   GPS thread priority.
*/
#define GPS_HANDLER_PRIO        (THREAD_PRIORITY_MAIN - 1) 
static kernel_pid_t gps_handler_pid;
static char gps_handler_stack[THREAD_STACKSIZE_MAIN];

#ifndef UART_BUFSIZE
#define UART_BUFSIZE        (128U)
#endif

/** 
*   Size of lorawan ringbuffer. 
*   It stores latitude, longitude and timestamp (Epoch) 4 times. 
*/
#ifndef LORAWAN_BUFSIZE
#define LORAWAN_BUFSIZE        (53U) 
#endif

/** 
*   Messages are sent every 300s (5 minutes). 20s is te minimum to respect the duty cycle on each channel 
*/
#define PERIOD_S            (300U)

/** 
*   Priority of the Lora thread. Must be lower than the GPS 
*/
#define SENDER_PRIO         (THREAD_PRIORITY_IDLE - 1) 
static kernel_pid_t sender_pid;
static char sender_stack[THREAD_STACKSIZE_MAIN / 2];
static void *sender(void *arg);

typedef struct {
    char rx_mem[LORAWAN_BUFSIZE];
    ringbuffer_t rx_buf;
} lora_ctx_t;
static lora_ctx_t ctx_lora;

semtech_loramac_t loramac;
static sx127x_t sx127x;

static ztimer_t timer;

char message[100];

static uint8_t deveui[LORAMAC_DEVEUI_LEN];
static uint8_t appeui[LORAMAC_APPEUI_LEN];

static uint8_t appskey[LORAMAC_APPSKEY_LEN];
static uint8_t nwkskey[LORAMAC_NWKSKEY_LEN];
static uint8_t devaddr[LORAMAC_DEVADDR_LEN];

static void _alarm_cb(void *arg) {
   (void) arg;
   msg_t msg;
   msg_send(&msg, sender_pid);
}

static void _prepare_next_alarm(void) {
   timer.callback = _alarm_cb;
   ztimer_set(ZTIMER_MSEC, &timer, PERIOD_S * MS_PER_SEC);
}


static void _send_message(void) {

    //MUDAR 27 PARA UMA CONSTANTE

    char *destination = (char*)malloc((26*2+1)*sizeof(char));
    ringbuffer_get(&(ctx_lora.rx_buf), destination, 27);

    /* Try to send the message */
    uint8_t ret = semtech_loramac_send(&loramac,(uint8_t*)(destination), strlen(destination));
    if (ret != SEMTECH_LORAMAC_TX_DONE)  {
        printf("Cannot send message '%s', ret code: %d\n\n", message, ret);
    }

    free(destination);
}

static void *sender(void *arg) {

    (void)arg;

    msg_t msg;
    msg_t msg_queue[8];
    msg_init_queue(msg_queue, 8);

    while (1) {
        msg_receive(&msg);       

       /* Trigger the message send */
       _send_message();

       /* Schedule the next wake-up alarm */
       _prepare_next_alarm();
    }

   /* this should never be reached */
   return NULL;
}

typedef struct {
    char rx_mem[UART_BUFSIZE];
    ringbuffer_t rx_buf;
} uart_ctx_t;
static uart_ctx_t ctx;


void rx_cb(void *arg, uint8_t data)
{
    uart_t dev = (uart_t)(uintptr_t)arg;

    ringbuffer_add_one(&ctx.rx_buf, data);

    if (data == '\n') {
        msg_t msg;
        msg.content.value = (uint32_t)dev;
        msg_send(&msg, gps_handler_pid);
    }
}

static void fix_minmea_sentence(char *line) {
    
    // Line must not be null
    if (line == NULL) 
        return;

    // Runs the string until a terminator char is found or it reaches its end '\n'
    while (*line != '\0') {
        if (*line == '\n') {
            // Substitues all values after '\n' for '\0'
            line++;
            while (*line != '\0') {
                *line = '\0';
                line++;
            }
            return;
        }
        line++;
    }
}

struct tm *tm;

/**
*   Writes date, time, speed and additonal data into desired ringbuffer
*/
static void store_data(struct minmea_sentence_rmc *frame, lora_ctx_t *ring) { //struct minmea_sentence_rmc *frame, struct tm *time, lora_ctx_t *ring
    

    
    //int j;
    //char p_datetime[8];
    //p_datetime = (char*)DATETIME;
    

    /* Char arrays used to store desired GPS data */
    size_t TEMP_SIZE = 26*2+1;
    char *temp = (char*)malloc(TEMP_SIZE*sizeof(char));   


    //char *package = (char*)malloc(53*sizeof(char));

    /* Gets latitude, longitude, date, time and speed */
    /* Latitude and longitude in absolute values */
    float latitude =  fabs(minmea_tocoord(&(*frame).latitude)); 
    float longitude = fabs(minmea_tocoord(&(*frame).longitude));
    float speed = minmea_tofloat(&(*frame).speed);
    minmea_getdatetime(tm, &((*frame).date), &((*frame).time));  

    

    /* Guarantees a nan value is not writen */
    if(isnan(latitude) && isnan(longitude)) {
        latitude = 90; // -90 before
        longitude = 90; 
    }    

    /* Conveting data into bytes and desired format */
    for(size_t i = 0; i < LORAMAC_DEVADDR_LEN; i++)
        DEVID[i] = (char) devaddr[i];

    /* Date and time */
    DATETIME[0] = (char) tm->tm_mday;
    DATETIME[1] = (char) tm->tm_mon;
    DATETIME[2] = (char) ((tm->tm_year + 1900) % 100);
    DATETIME[3] = (char) tm->tm_hour;
    DATETIME[4] = (char) tm->tm_min;
    DATETIME[5] = (char) tm->tm_sec; 

    /* Latitude */
    int decimal = (int) floor(latitude);
    snprintf(&LATITUDE[0], 3, "%d", decimal);

    latitude = latitude - decimal;
    latitude = latitude * 1000000;

    LATITUDE[3] = (char) ((int)latitude % 100);
    latitude = (int) latitude % 10000;
    LATITUDE[2] = (char) ((int)latitude % 100);
    latitude = (int) latitude % 100;
    LATITUDE[1] = (char) ((int)latitude % 100);

    /* Longitude */
    decimal = (int) floor(longitude);
    snprintf(&LONGITUDE[0], 3, "%d", decimal);

    longitude = longitude - decimal;
    longitude = longitude * 1000000;

    LONGITUDE[3] = (char) ((int)longitude % 100);
    longitude = (int)longitude % 10000;
    LONGITUDE[2] = (char) ((int)longitude % 100);
    longitude = (int)longitude % 100;
    LONGITUDE[1] = (char) ((int)longitude % 100);

    /* Speed */
    decimal = (int) fabs(speed);
    snprintf(&SPEED[0], 3, "%c", (char) decimal);

    speed = speed - decimal;
    speed = speed * 10000;

    SPEED[2] = (char) ((int)speed % 100);
    speed = (int)speed % 100;
    SPEED[1] = (char) ((int)speed % 100);

    /* Final string */
    snprintf(temp, TEMP_SIZE, "%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c\n", DEFAULT, DEVID[0],
        DEVID[1], DEVID[2], DEVID[3], DEFAULT, DEFAULT, DEFAULT, DATETIME[0], DATETIME[1], DATETIME[2],
        DATETIME[3], DATETIME[4], DATETIME[5], LATITUDE[0], LATITUDE[1], LATITUDE[2], LATITUDE[3],
        LONGITUDE[0], LONGITUDE[1], LONGITUDE[2], LONGITUDE[3], SPEED[0], SPEED[1], SPEED[2]);

    /* Checking final string */
    printf("String final: ");
    for (size_t i = 0; i <= TEMP_SIZE; i++)
        printf("%d", temp[i]);


    //printf("HDR: %x \r\n", HDR);
    //printf("DEVID: %010lx \r\n", DEVID);
    //printf("DEFAULT: %02x \r\n", DEFAULT);
    //printf("DATETIME: %12lx \r\n", DATETIME);
    
    //for (uint8_t j=0; j<(strlen(DATETIME)); j++)
    //    printf("%02X", DATETIME[j]);
        
    //printf("LATITUDE: %08lx \r\n", LATITUDE);
    //printf("LONGITUDE: %08lx \r\n", LONGITUDE);
    //printf("SPEED: %06lx \r\n", SPEED);

    /* 53 is for all the algarisms, ponctuations (-,;+ ...) and the null character */

    //size_t TESTE_SIZE = 24;
    //char *teste = (char*)malloc(TESTE_SIZE*sizeof(char));

    //int ret; 
    //snprintf(temp, 53, "%02x", HDR);
    //for (size_t i=0; i<4; i++)
    //    snprintf(temp, 53, "%02x", DEVID[i]);
    //snprintf(temp, 7, "%02x%02x%02x", DEFAULT, DEFAULT, DEFAULT);

    //size_t index = 0;
    //teste[index] = HDR; index++;
    //for (size_t j=0; j<sizeof(DATETIME); j++)
    //    teste[j] = DATETIME[j];


    //for (size_t i=0; i<3; i++)
    //    snprintf(temp, 53, "%02x", LATITUDE[i]);
    //for (size_t i=0; i<3; i++)
    //    snprintf(temp, 53, "%02x", LONGITUDE[i]);
    //for (size_t i=0; i<2; i++)
    //    snprintf(temp, 53, "%02x", SPEED[i]);

    //int ret = snprintf(temp,53,"%02x%10s%02x%02x%02x%12s%8s%8s%6s", HDR, DEVID, DEFAULT, DEFAULT, DEFAULT, DATETIME, LATITUDE,
    //                    LONGITUDE, SPEED); 

    //if (ret < 0) {
    //    puts("Cannot get gps data");
    //    return;
    //}

    //printf("SIZE TESTE: %d\r\n", sizeof(teste));

    
    



    /* Strcpy is necessary to add the null character in the end of array */
    //strcpy(package, temp);

    /* It has to be done in a for loop to overwrite old data if ringbuffer is full */
    for(size_t i = 0; i <= TEMP_SIZE; i++) {
        ringbuffer_add_one(&(*ring).rx_buf, temp[i]);
    }
    
    free(temp);
    
    //free(temp);
    //free(package);  

    puts("Data stored =)\r\n");                       
}

static void *gps_handler(void *arg)
{
    (void)arg;
    msg_t msg;
    msg_t msg_queue[8];
    msg_init_queue(msg_queue, 8);

    int pos = 0;
    char line[MINMEA_MAX_SENTENCE_LENGTH];  // char line[MINMEA_MAX_SENTENCE_LENGTH];

    while (1) {

        msg_receive(&msg);

        char c;

        do {
            c = (char)ringbuffer_get_one(&(ctx.rx_buf));

            if (c == '\n') {

                line[pos++] = c;
		        pos = 0;
                
                printf("LINE ANTES: %s\r\n", line);

                fix_minmea_sentence(line); 

                printf("LINE DEPOIS: %s\r\n", line);           

                switch (minmea_sentence_id(line, false)) {

                    case MINMEA_SENTENCE_RMC: {

                        struct minmea_sentence_rmc frame;

                        if (minmea_parse_rmc(&frame, line)) {    
                            printf("$RMC floating point degree coordinates and speed: (%f,%f) %f\r\n",
                                    minmea_tocoord(&frame.latitude),
                                    minmea_tocoord(&frame.longitude),
                                    minmea_tofloat(&frame.speed));

                            store_data(&frame, &ctx_lora); 

                        } else {
                            puts("Could not parse $RMC message. Possibly incomplete");
                        }
                    } break;

                    case MINMEA_SENTENCE_GGA: {
                        struct minmea_sentence_gga frame;
                        if (minmea_parse_gga(&frame, line)) {
                            printf("$GGA: fix quality: %d\n", frame.fix_quality);
                        }
                    } break;

                    case MINMEA_SENTENCE_GSV: {
                        struct minmea_sentence_gsv frame;
                        if (minmea_parse_gsv(&frame, line)) {
                            printf("$GSV: message %d of %d\n", frame.msg_nr, frame.total_msgs);
                            printf("$GSV: sattelites in view: %d\n", frame.total_sats);
                            for (int i = 0; i < 4; i++)
                                printf("$GSV: sat nr %d, elevation: %d, azimuth: %d, snr: %d dbm\n",
                                    frame.sats[i].nr,
                                    frame.sats[i].elevation,
                                    frame.sats[i].azimuth,
                                    frame.sats[i].snr);
                        }
                    }break;                    
                    
                    default : {
                        printf("Could not parse any message!\r\n");
                    } break;
                }
            }
            else {
                line[pos++] = c;
            }
        } while (c != '\n');
    }

    /* This should never be reached */
    return NULL;
}

int init_gps(void)
{

    /* Initialize UART */
    int dev = 1;
    uint32_t baud = 9600;

    int res = uart_init(UART_DEV(dev), baud, rx_cb, (void *)dev);
    if (res != UART_OK) {
        puts("Error: Unable to initialize UART device");
        return 1;
    }
    printf("Success: Initialized UART_DEV(%i) at BAUD %"PRIu32"\n", dev, baud);

    /* Tell gps chip to wake up */
    uart_write(UART_DEV(dev), (uint8_t *)PMTK_SET_NMEA_OUTPUT_RMC, strlen(PMTK_SET_NMEA_OUTPUT_RMC));
    uart_write(UART_DEV(dev), (uint8_t *)PMTK_SET_UPDATE_F_2HZ, strlen(PMTK_SET_UPDATE_F_2HZ));
    puts("GPS Started.");
    return 0;
}


int main(void) {

    puts("\nRIOT ADC peripheral driver test\n");
    puts("This test will sample all available ADC lines once every 100ms with\n"
         "a 10-bit resolution and print the sampled results to STDIO\n\n");
    
    
    /* Initialize gps ringbuffer */ 
    ringbuffer_init(&(ctx.rx_buf), ctx.rx_mem, UART_BUFSIZE);

    /* Initialize lora ringbuffer */
    ringbuffer_init(&(ctx_lora.rx_buf), ctx_lora.rx_mem, LORAWAN_BUFSIZE);

    init_gps();
    
    /* Start the gps_handler thread */
    gps_handler_pid = thread_create(gps_handler_stack, sizeof(gps_handler_stack), GPS_HANDLER_PRIO, 0, gps_handler, NULL, "gps_handler");

    puts("LoRaWAN Class A low-power application");
    puts("=====================================");

    /* Convert identifiers and application key */
    fmt_hex_bytes(deveui, CONFIG_LORAMAC_DEV_EUI_DEFAULT);
    fmt_hex_bytes(appeui, CONFIG_LORAMAC_APP_EUI_DEFAULT);

    fmt_hex_bytes(devaddr, CONFIG_LORAMAC_DEV_ADDR_DEFAULT);
    fmt_hex_bytes(nwkskey, CONFIG_LORAMAC_NWK_SKEY_DEFAULT);
    fmt_hex_bytes(appskey, CONFIG_LORAMAC_APP_SKEY_DEFAULT);

    /* Initialize the radio driver */
    sx127x_setup(&sx127x, &sx127x_params[0], 0);
    loramac.netdev = &sx127x.netdev;
    loramac.netdev->driver = &sx127x_driver;

    /* Initialize the loramac stack */
    semtech_loramac_init(&loramac);
    semtech_loramac_set_deveui(&loramac, deveui);
    semtech_loramac_set_appeui(&loramac, appeui);
    semtech_loramac_set_devaddr(&loramac, devaddr);
    semtech_loramac_set_appskey(&loramac, appskey);
    semtech_loramac_set_nwkskey(&loramac, nwkskey);

    /* Set a channels mask that makes it use only the first 8 channels */
    uint16_t channel_mask[LORAMAC_CHANNELS_MASK_LEN] = { 0 };
    channel_mask[0] = 0x00FF;
    semtech_loramac_set_channels_mask(&loramac, channel_mask);

    /* Use a fast datarate */
    semtech_loramac_set_dr(&loramac, LORAMAC_DR_5);

    /* Start the ABP procedure */
    puts("Starting join procedure");
    if (semtech_loramac_join(&loramac, LORAMAC_JOIN_ABP) != SEMTECH_LORAMAC_JOIN_SUCCEEDED) {
        puts("Join procedure failed");
        return 1;
    }
    puts("Join procedure succeeded");

    /* Start the Lora thread */
    sender_pid = thread_create(sender_stack, sizeof(sender_stack), SENDER_PRIO, 0, sender, NULL, "sender");

    /* Wakes the Lora thread up */
    msg_t msg;
    msg_send(&msg, sender_pid);

    return 0;
}