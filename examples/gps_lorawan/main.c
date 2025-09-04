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
#include <time.h>

#include "bq2429x.h"

#include "assert.h"
#include "event/timeout.h"

#include "msg.h"
#include "thread.h"
#include "fmt.h"

#include "net/loramac.h"
#include "semtech_loramac.h"
#include "net/netdev/lora.h"
#include "net/netdev.h"
#include "net/lora.h"

#include "sx127x.h"
#include "sx127x_internal.h"
#include "sx127x_netdev.h"
#include "sx127x_params.h"

#include "periph/gpio.h"
#include "xtimer.h"

#include <inttypes.h>

#include "board.h"
#include "periph/adc.h"
#include "periph/pm.h"

#include "ringbuffer.h"
#include "periph/uart.h"
#include "minmea.h"

#include "ztimer.h"
#include "shell.h"
#include "sched.h"
#include "log.h"

#define PMTK_SET_NMEA_OUTPUT_RMC    "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"
#define PMTK_SET_UPDATE_F_2HZ       "$PMTK300,500,0,0,0,0*28\r\n"

//Constantes

#define DEVID_LEN 5U
#define DATE_TIME_LEN 6U
#define LATITUDE_LEN 4U
#define LONGITUDE_LEN 4U
#define SPEED_LEN 3U
#define LORA_MSG_LEN 51U 

// LORA_MSM_LEN eh o total de digitos



//const size_t DATE_TIME_LEN = 6;
//const size_t LATITUDE_LEN = 4;
//const size_t LONGITUDE_LEN = 4;
//const size_t SPEED_LEN = 3;

uint8_t HDR = 0x10;
uint8_t DEVID[DEVID_LEN] = {0x00,0x00,0x00,0x00,0x00};
uint8_t MODEL[1] = {0x12};
uint8_t SW_VER[2] = {0x01,0x35};
uint8_t DATETIME[DATE_TIME_LEN];
uint8_t LATITUDE[LATITUDE_LEN]; 
uint8_t LONGITUDE[LONGITUDE_LEN];
uint8_t SPEED[SPEED_LEN];

uint8_t lora_msg[LORA_MSG_LEN];

//char p_datetime[8];
//p_datetime = (char *)&DATETIME;




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
#define LORAWAN_BUFSIZE        (51U) 
#endif

/** 
*   Messages are sent every 300s (5 minutes). 20s is te minimum to respect the duty cycle on each channel 
*/
#define PERIOD_S            (20U)

/** 
*   Priority of the Lora thread. Must be lower than the GPS 
*/
#define SENDER_PRIO         (THREAD_PRIORITY_IDLE - 1) 
static kernel_pid_t sender_pid;
static char sender_stack[THREAD_STACKSIZE_MAIN / 2];
static void *sender(void *arg);

//typedef struct {
//    char rx_mem[LORAWAN_BUFSIZE];
//    ringbuffer_t rx_buf;
//} lora_ctx_t;
//static lora_ctx_t ctx_lora;

semtech_loramac_t loramac;
static sx127x_t sx127x;

static ztimer_t timer;

char message[100];

static uint8_t deveui[LORAMAC_DEVEUI_LEN];
static uint8_t appeui[LORAMAC_APPEUI_LEN];

static uint8_t appskey[LORAMAC_APPSKEY_LEN];
static uint8_t nwkskey[LORAMAC_NWKSKEY_LEN];
static uint8_t devaddr[LORAMAC_DEVADDR_LEN];

int reset_lora_driver(void)
{
    netdev_t *netdev = &sx127x.netdev;

    printf("[lora] resetting sx127x\n");
    netopt_state_t state = NETOPT_STATE_RESET;

    netdev->driver->set(netdev, NETOPT_STATE, &state, sizeof(netopt_state_t));
    return 0;
}

static void _event_cb(netdev_t *dev, netdev_event_t event)
{
    (void) dev;
    switch (event) {

    case NETDEV_EVENT_TX_COMPLETE:
        sx127x_set_sleep(&sx127x);
        printf("[lora] transmission completed");
        break;

    case NETDEV_EVENT_CAD_DONE:
        break;

    case NETDEV_EVENT_TX_TIMEOUT:
        sx127x_set_sleep(&sx127x);
        break;

    default:
        printf("[lora] unexpected netdev event received: %d\n", event);
        break;
    }
}

int init_lora_driver(void)
{
    sx127x.params = sx127x_params[0];
    netdev_t *netdev = &sx127x.netdev;

    netdev->driver = &sx127x_driver;

    if (netdev->driver->init(netdev) < 0) {
        printf("[lora] failed to initialize SX127x device, exiting");
        return 1;
    }

    netdev->event_callback = _event_cb;
    return 0;
}

void init_lorawan(void)
{
    printf("[lora] initializing lorawan...\n");

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

    semtech_loramac_set_adr(&loramac, false);
    semtech_loramac_set_dr(&loramac, LORAMAC_DR_5);
    printf("[lora] lorawan initialized with DEV_EUI=%s\n", CONFIG_LORAMAC_DEV_EUI_DEFAULT);
}

bool joined = false;
int join_lorawan(void)
{
    printf("[lora] starting join procedure\n");
    if (semtech_loramac_join(&loramac, LORAMAC_JOIN_ABP) != SEMTECH_LORAMAC_JOIN_SUCCEEDED) {
        printf("[lora] join procedure failed\n");
        return 1;
    }
    joined = true;
    printf("[lora] join procedure succeeded\n");
    return 0;
}

uint8_t send_count;
int enable_lorawan(void)
{
    printf("[lora] enable_lorawan\n");
    reset_lora_driver();
    init_lora_driver();
    init_lorawan();
    send_count = 0;
    return 0;
}

static void _alarm_cb(void *arg) {
   (void) arg;
   msg_t msg;
   msg_send(&msg, sender_pid);
}

static void _prepare_next_alarm(void) {
   timer.callback = _alarm_cb;
   ztimer_set(ZTIMER_MSEC, &timer, PERIOD_S * MS_PER_SEC);
}


static uint8_t _send_message(void) {

    //char *lora_msg = (char*)malloc(LORA_MSG_LEN*sizeof(char));
    //ringbuffer_get(&(ctx_lora.rx_buf), lora_msg, LORA_MSG_LEN);

    printf("DESTINATION: \n\n\n\n\r");
    for(size_t i=0; i<LORA_MSG_LEN; i++)
        printf("%02x", lora_msg[i]);

    /* Try to send the message */
    uint8_t lora_ret = semtech_loramac_send(&loramac,(uint8_t*)(lora_msg), LORA_MSG_LEN);
    uint8_t ret;
    switch (lora_ret) {
        case SEMTECH_LORAMAC_NOT_JOINED:
            printf("[lora] cannot send: not joined\n");
            joined = false;
            ret = 2; // join and re-send message
            break;
        case SEMTECH_LORAMAC_BUSY:
            printf("[lora] cannot send: MAC is busy\n");
            ret = 3; // wait and re-send message
            break;
        case SEMTECH_LORAMAC_DUTYCYCLE_RESTRICTED:
            printf("[lora] cannot send: dutycycle restriction\n");
            ret = 1;
            break;
        case SEMTECH_LORAMAC_TX_ERROR:
            printf("[lora] cannot send: error\n");
            ret = 1;
            break;
        case SEMTECH_LORAMAC_TX_CNF_FAILED:
            printf("[lora] fail to send: no ACK received\n");
            ret = 1;
            break;
        default:
            ret = 0;
            printf("[lora] success sending message\n");
            printf("[lora enum] %d \n", lora_ret);
            break;
    }
    send_count++;
    return ret;

    //free(lora_msg);
}

static void *sender(void *arg) {

    (void)arg;

    msg_t msg;
    msg_t msg_queue[8];
    msg_init_queue(msg_queue, 8);
    uint8_t ret;

    while (1) {
        msg_receive(&msg);       

        if(!joined){
            enable_lorawan();
            if(join_lorawan() == 0){
                _send_message();
            }
        }
        else{
            ret = _send_message();;
            if(ret == 2){
                enable_lorawan();
                if(join_lorawan()== 0)
                    _send_message();
            }
            else if(ret == 3){
                ztimer_sleep(ZTIMER_MSEC, 3);
                _send_message();
            }
        }

        /* Schedule the next wake-up alarm */
        _prepare_next_alarm();

        if(send_count >= 10) {
            enable_lorawan();
        }
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

/* Fixes the minmea received sentence, excluding possible trash data */
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

//struct tm *tm;

/**
*   Writes date, time, speed and additonal data into desired ringbuffer
*/
static void store_data(struct minmea_sentence_rmc *frame) { //struct minmea_sentence_rmc *frame, struct tm *time, lora_ctx_t *ring
    
    
    
    //int j;
    //char p_datetime[8];
    //p_datetime = (char*)DATETIME;
    

    /* Char arrays used to store desired GPS data */
    // +1 for null term
    //uint8_t *lora_msg = (uint8_t*)malloc((LORA_MSG_LEN+1)*sizeof(uint8_t));   


    //char *package = (char*)malloc(53*sizeof(char));

    /* Gets latitude, longitude, date, time and speed */
    /* Latitude and longitude in absolute values */

    float latitude =  minmea_tocoord(&(*frame).latitude); 
    float longitude = minmea_tocoord(&(*frame).longitude);
    float speed = minmea_tofloat(&(*frame).speed);

    /* Guarantees a nan value is not writen */
    if(isnan(latitude) && isnan(longitude)) {
        latitude = 90; // -90 before
        longitude = 90; 
    }   

    if (latitude < 0) latitude = -latitude;
    if (longitude < 0) longitude = -longitude;

    //float latitude =  abs(minmea_tocoord(&(*frame).latitude)); 
    //float longitude = abs(minmea_tocoord(&(*frame).longitude));
    //float speed = minmea_tofloat(&(*frame).speed);
    //minmea_getdatetime(tm, &(frame->date), &(frame->time));  

    

     

    /* Conveting data into bytes and desired format */
    for(size_t i = 0; i < 4; i++)
        DEVID[i] = (deveui[i]/10)*16 + devaddr[i]%10;

    // Printing DEVID
    printf("DEVID: ");
    for(size_t i = 0; i < LORAMAC_DEVADDR_LEN; i++)
        printf("%x", DEVID[i]);

    puts("");

    // Printing DEVID from source
    printf("DEVID(source): ");
    for(size_t i = 0; i < LORAMAC_DEVADDR_LEN; i++)
        printf("%x", ((deveui[i]/10)*16 + deveui[i]%10));

    puts("");

    /* Date and time */
    DATETIME[0] = frame->date.year;
    DATETIME[1] = frame->date.month;
    DATETIME[2] = frame->date.day;
    DATETIME[3] = frame->time.hours - 3; // UTC -3
    DATETIME[4] = frame->time.minutes;
    DATETIME[5] = frame->time.seconds; 

    //printf("DATETIME(foolproof): %d;%d;%d;%d;%d;%d\n\r", tm->tm_mday, tm->tm_mon, ((tm->tm_year + 1900) % 100), tm->tm_hour, tm->tm_min, tm->tm_sec);

    /* Latitude */
    printf("latitude1: %f\r\n", latitude);
    uint8_t lat_int_part = (uint8_t) floor(latitude);
    LATITUDE[0] = lat_int_part;
    //snprintf(&LATITUDE[0], 3, "%X", (char) decimal);

    printf("latitude2: %f\r\n", latitude);
    latitude = latitude - lat_int_part;
    printf("lat3: %f\r\n", latitude);
    uint32_t lat = (int) floor(latitude * 1000000);
    printf("lat4: %ld\r\n", lat);

    uint8_t aux_lat = lat /10000; // first two decimal digits
    LATITUDE[1] =  (aux_lat/10)*16 + aux_lat%10;
    printf("guardado: %02X \r\n", ((aux_lat/10)*16 + aux_lat%10));
    aux_lat = (lat % 10000) /100; // middle two decimal digits
    LATITUDE[2] =  (aux_lat/10)*16 + aux_lat%10;
    printf("guardado: %02X \r\n", ((aux_lat/10)*16 + aux_lat%10));
    aux_lat = lat % 100; // last two decimal digits
    LATITUDE[3] =  (aux_lat/10)*16 + aux_lat%10;
    printf("guardado: %02X \r\n", ((aux_lat/10)*16 + aux_lat%10));

    /* Longitude */
    uint8_t lon_int_part = (int) floor(longitude);
    LONGITUDE[0] = lon_int_part;
    //snprintf(&LONGITUDE[0], 3, "%X", (char) decimal);

    longitude = longitude - lon_int_part;
    uint32_t lon = (int) floor(longitude * 1000000);

    uint8_t aux_lon = lon /10000; // first two decimal digits
    LONGITUDE[1] =  (aux_lon/10)*16 + aux_lon%10;
    aux_lon = (lon % 10000) /100; // middle two decimal digits
    LONGITUDE[2] =  (aux_lon/10)*16 + aux_lon%10;
    aux_lon = lon % 100; // last two decimal digits
    LONGITUDE[3] =  (aux_lon/10)*16 + aux_lon%10;

    /* Speed */
    uint16_t spd_int_part = (int) floor(speed);
    SPEED[0] = spd_int_part / 256 ;
    SPEED[1] = spd_int_part % 256 ;

    speed = speed - spd_int_part;
    uint8_t aux_speed = (int) floor(speed * 100);

    SPEED[2] =  (aux_speed/10)*16 + aux_speed%10;


    //printf("HDR: %x \r\n", HDR);
    //printf("DEVID: %010lx \r\n", DEVID);
    //printf("DEFAULT: %02x \r\n", DEFAULT);
    //printf("DATETIME: %12lx \r\n", DATETIME);

    printf("DATETIME: ");
    
    for (uint8_t j=0; j<DATE_TIME_LEN; j++)
        printf("%02x;", DATETIME[j]);

    puts("");



    printf("LATITIDE: ");
    printf("%02X;", LATITUDE[0]);
    for (uint8_t j=1; j<LATITUDE_LEN; j++)
        printf("%02x;", LATITUDE[j]);

    puts("");

        

    printf("LONGITUDE: ");
    printf("%02X;", LONGITUDE[0]);
    for (uint8_t j=1; j<LONGITUDE_LEN; j++)
        printf("%02x;", LONGITUDE[j]);

    puts("");

    printf("SPEED: ");
    for (uint8_t j=0; j<SPEED_LEN; j++)
        printf("%02x;", SPEED[j]);

    puts("");

    lora_msg[0] = HDR; 
    lora_msg[1] = DEVID[0]; lora_msg[2] = DEVID[1]; lora_msg[3] = DEVID[2]; lora_msg[4] = DEVID[3]; lora_msg[5] = DEVID[4];
    lora_msg[6] = MODEL[0]; lora_msg[7] = SW_VER[0]; lora_msg[8] = SW_VER[1];
    lora_msg[9] = DATETIME[0]; lora_msg[10] = DATETIME[1]; lora_msg[11] = DATETIME[2];
    lora_msg[12] = DATETIME[3]; lora_msg[13] = DATETIME[4]; lora_msg[14] = DATETIME[5]; 
    lora_msg[15] = LATITUDE[0]; lora_msg[16] = LATITUDE[1]; lora_msg[17] = LATITUDE[2]; lora_msg[18] = LATITUDE[3];
    lora_msg[19] = LONGITUDE[0]; lora_msg[20] = LONGITUDE[1]; lora_msg[21] = LONGITUDE[2]; lora_msg[22] = LONGITUDE[3];
    lora_msg[23] = SPEED[0]; lora_msg[24] = SPEED[1]; lora_msg[25] = SPEED[2];

    for (uint8_t j=26; j<LORA_MSG_LEN; j++)
        lora_msg[j] = 0x00;

    lora_msg[29] = 0xe9;

    lora_msg[LORA_MSG_LEN] = '\n';




    //snprintf(lora_msg, LORA_MSG_LEN, "%X%X%X%X%X%X%X%X%d%d%d%d%d%d%X%d%d%d%X%d%d%d%d%d%d", DEFAULT, 
    //        DEVID[0],DEVID[1], DEVID[2], DEVID[3], 
    //        DEFAULT, DEFAULT, DEFAULT, 
    //        (int) DATETIME[0], (int) DATETIME[1], (int) DATETIME[2], (int) DATETIME[3], (int) DATETIME[4], (int) DATETIME[5],
    //        LATITUDE[0], (int) LATITUDE[1], (int) LATITUDE[2], (int) LATITUDE[3],
    //        LONGITUDE[0], (int) LONGITUDE[1], (int) LONGITUDE[2], (int) LONGITUDE[3],
    //        (int) SPEED[0], (int) SPEED[1], (int) SPEED[2]);

    
    printf("String enviada: ");
    for (size_t j=0; j<LORA_MSG_LEN; j++)
        printf("%02x;", lora_msg[j]);

    puts("");
        
        
    //printf("LATITUDE: %08lx \r\n", LATITUDE);
    //printf("LONGITUDE: %08lx \r\n", LONGITUDE);
    //printf("SPEED: %06lx \r\n", SPEED);

    /* 53 is for all the algarisms, ponctuations (-,;+ ...) and the null character */

    //size_t TESTE_SIZE = 7;
    //char *teste = (char*)malloc(TESTE_SIZE*sizeof(char));

    //int ret; 
    //snprintf(lora_msg, 53, "%02x", HDR);
    //for (size_t i=0; i<4; i++)
    //    snprintf(lora_msg, 53, "%02x", DEVID[i]);
    //snprintf(lora_msg, 7, "%02x%02x%02x", DEFAULT, DEFAULT, DEFAULT);

    //size_t index = 0;
    //teste[index] = HDR; index++;
    //for (size_t j=0; j<sizeof(DATETIME); j++)
    //    teste[j] = DATETIME[j];


    //for (size_t i=0; i<3; i++)
    //    snprintf(lora_msg, 53, "%02x", LATITUDE[i]);
    //for (size_t i=0; i<3; i++)
    //    snprintf(lora_msg, 53, "%02x", LONGITUDE[i]);
    //for (size_t i=0; i<2; i++)
    //    snprintf(lora_msg, 53, "%02x", SPEED[i]);

    //int ret = snprintf(lora_msg,53,"%02x%10s%02x%02x%02x%12s%8s%8s%6s", HDR, DEVID, DEFAULT, DEFAULT, DEFAULT, DATETIME, LATITUDE,
    //                    LONGITUDE, SPEED); 

    //if (ret < 0) {
    //    puts("Cannot get gps data");
    //    return;
    //}

    //printf("SIZE TESTE: %d\r\n", sizeof(teste));

    //printf("String final: ");
    //for (size_t i=0; i<TESTE_SIZE; i++)
    //    printf("%02x", teste[i]);

    



    /* Strcpy is necessary to add the null character in the end of array */
    //strcpy(package, lora_msg);

    /* It has to be done in a for loop to overwrite old data if ringbuffer is full */
    //for(uint8_t i = 0; i <= LORA_MSG_LEN; i++) {
    //    ringbuffer_add_one(&(*ring).rx_buf, lora_msg[i]);
    //}
    
    //free(teste);
    
    //free(lora_msg);
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
                
                //printf("LINE ANTES: %s\r\n", line);

                fix_minmea_sentence(line); 

                //printf("LINE DEPOIS: %s\r\n", line);           

                switch (minmea_sentence_id(line, false)) {

                    case MINMEA_SENTENCE_RMC: {

                        struct minmea_sentence_rmc frame;

                        if (minmea_parse_rmc(&frame, line)) {    
                            printf("$RMC floating point degree coordinates and speed: (%f,%f) %f\r\n",
                                    minmea_tocoord(&frame.latitude),
                                    minmea_tocoord(&frame.longitude),
                                    minmea_tofloat(&frame.speed));

                            printf("DATETIME(gps_handler): %d;%d;%d;%d;%d;%d", frame.date.day, frame.date.month, frame.date.year, frame.time.hours, frame.time.minutes,
                                frame.time.seconds);

                            store_data(&frame); 

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
    //ringbuffer_init(&(ctx_lora.rx_buf), ctx_lora.rx_mem, LORAWAN_BUFSIZE);

    init_gps();
    
    /* Start the gps_handler thread */
    gps_handler_pid = thread_create(gps_handler_stack, sizeof(gps_handler_stack), GPS_HANDLER_PRIO, 0, gps_handler, NULL, "gps_handler");

    puts("LoRaWAN Class A low-power application");
    puts("=====================================");

    init_lora_driver();
    enable_lorawan();
    join_lorawan();

    /* Start the Lora thread */
    sender_pid = thread_create(sender_stack, sizeof(sender_stack), SENDER_PRIO, 0, sender, NULL, "sender");

    /* Wakes the Lora thread up */
    msg_t msg;
    msg_send(&msg, sender_pid);

    return 0;
}