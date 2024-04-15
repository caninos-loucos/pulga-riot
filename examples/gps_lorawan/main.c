/*
 * Copyright (C) 2019 Freie Universität Berlin
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
 * @brief       (Mock-up) BLE heart rate sensor example
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Hendrik van Essen <hendrik.ve@fu-berlin.de>
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

#include "assert.h"
#include "event/timeout.h"
//#include "nimble_riot.h"
//#include "nimble_autoadv.h"
//#include "net/bluetil/ad.h"

//#include "host/ble_hs.h"
//#include "host/ble_gatt.h"
//#include "services/gap/ble_svc_gap.h"
//#include "services/gatt/ble_svc_gatt.h"

//#include "bmx280.h"
//#include "bmx280_params.h"

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
//#include "scd30.h"
//#include "scd30_params.h"
//#include "scd30_internal.h"

#include <inttypes.h>

//#include "si1133.h"
//#include "si1133_params.h"
#include "board.h"
#include "periph/adc.h"

//#include "shell.h"
//#include "shell_commands.h" 

#include "ringbuffer.h"
#include "periph/uart.h"
#include "minmea.h"

// Inlcude do led

//#include "led.h" 

//// Fim LED

#define PMTK_SET_NMEA_OUTPUT_RMC    "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"
#define PMTK_SET_UPDATE_F_2HZ       "$PMTK300,500,0,0,0,0*28\r\n"

#define GPS_HANDLER_PRIO        (THREAD_PRIORITY_MAIN - 1)
static kernel_pid_t gps_handler_pid;
static char gps_handler_stack[THREAD_STACKSIZE_MAIN];

#ifndef UART_BUFSIZE
#define UART_BUFSIZE        (128U)
#endif

/* Ringbuffer enviado por Lorawan */
#ifndef LORAWAN_BUFSIZE
#define LORAWAN_BUFSIZE        (128)
#endif


///* Get GPS readings */
//float *latitude;
//float *longitude;
//float *speed;

///* Helper macro to define _si1133_strerr */
//#define CASE_SI1133_ERROR_STRING(X)                                            \/
//    case X:                                                                    \/
//        return #X;

//static const char *_si1133_strerr(si1133_ret_code_t err)
//{
//    switch (err) {
//        CASE_SI1133_ERROR_STRING(SI1133_OK);
//        CASE_SI1133_ERROR_STRING(SI1133_ERR_PARAMS);
//        CASE_SI1133_ERROR_STRING(SI1133_ERR_I2C);
//        CASE_SI1133_ERROR_STRING(SI1133_ERR_LOGIC);
//        CASE_SI1133_ERROR_STRING(SI1133_ERR_NODEV);
//        CASE_SI1133_ERROR_STRING(SI1133_ERR_OVERFLOW);
//    }
//    return NULL;
//}

//#define EXPECT_RET_CODE(expected, actual)                                      \/
//    do {                                                                       \/
//        si1133_ret_code_t actual_value = (actual);                             \/
//        si1133_ret_code_t expected_value = (expected);                         \/
//        if (actual_value != expected_value) {                                  \/
//            printf(                                                            \/
//                "ERROR: " #actual " = %s\nExpected value " #expected " (%s)\n",\/
//                _si1133_strerr(actual_value), _si1133_strerr(expected_value)); \/
//            failures++;                                                        \/
//        }                                                                      \/
//    } while (0)
//
//static si1133_t devSI;
//int32_t valuesSI11[3];

/* BLLE update interval */
#define UPDATE_INTERVAL         (250U) 

//static const char *_manufacturer_name = "Unfit Byte Inc.";
//static const char *_model_number = "2A";
//static const char *_serial_number = "a8b302c7f3-29183-x8";
//static const char *_fw_ver = "13.7.12";
//static const char *_hw_ver = "V3B";

//static event_queue_t _eq;
//static event_t _update_evt;
//static event_timeout_t _update_timeout_evt;

//static kernel_pid_t sender_pid;

/* Messages are sent every 20s to respect the duty cycle on each channel */
#define PERIOD_S            (20U)

#define SENDER_PRIO         (THREAD_PRIORITY_MAIN - 1)
static kernel_pid_t sender_pid;
static char sender_stack[THREAD_STACKSIZE_MAIN / 2];
static void *sender(void *arg);

//static bmx280_t dev;

//scd30_t scd30_dev;
//scd30_params_t params = SCD30_PARAMS;
//scd30_measurement_t result;

//static uint16_t _conn_handle;
//static uint16_t _hrs_val_handle;
//#define RES             ADC_RES_10BIT
//#define DELAY_MS        100U

//char sensor_measurements[100];  // Talvez desnecessário? Ou trocar o nome da variável?

//static int _devinfo_handler(uint16_t conn_handle, uint16_t attr_handle,
//                            struct ble_gatt_access_ctxt *ctxt, void *arg);
//
//static int gatt_svr_chr_access_rw_demo(
//    uint16_t conn_handle, uint16_t attr_handle,
//    struct ble_gatt_access_ctxt *ctxt, void *arg);
//
//static int send_latest_measurements(
//    uint16_t conn_handle, uint16_t attr_handle,
//    struct ble_gatt_access_ctxt *ctxt, void *arg)
//{
//    (void)conn_handle;
//    (void)attr_handle;
//    (void)arg;
//
//    int rc = os_mbuf_append(ctxt->om, sensor_measurements, sizeof(sensor_measurements));
//
//    return rc;
//}

//static void _start_updating(void);
//static void _stop_updating(void);
//
//static void _send_message(void);

///* UUID = 1bce38b3-d137-48ff-a13e-033e14c7a335 */
//static const ble_uuid128_t gatt_svr_svc_rw_demo_uuid
//        = BLE_UUID128_INIT(0x35, 0xa3, 0xc7, 0x14, 0x3e, 0x03, 0x3e, 0xa1, 0xff,
//                0x48, 0x37, 0xd1, 0xb3, 0x38, 0xce, 0x1b);
//
///* UUID = ccdd113f-40d5-4d68-86ac-a728dd82f4ab */
//static const ble_uuid128_t latest_mesurement_uuid
//        = BLE_UUID128_INIT(0xab, 0xf4, 0x82, 0xdd, 0x28, 0xa7, 0xac, 0x86, 0x68,
//                0x4d, 0xd5, 0x40, 0x3f, 0x11, 0xdd, 0xcc);
//
///* UUID = 35f28386-3070-4f3b-ba38-27507e991762 */
//static const ble_uuid128_t gatt_svr_chr_rw_demo_write_uuid
//        = BLE_UUID128_INIT(0x62, 0x17, 0x99, 0x7e, 0x50, 0x27, 0x38, 0xba, 0x3b,
//                0x4f, 0x70, 0x30, 0x86, 0x83, 0xf2, 0x35);
//
///* UUID = ccdd113f-40d5-4d68-86ac-a728dd82f4aa */
//static const ble_uuid128_t gatt_svr_chr_rw_demo_readonly_uuid
//        = BLE_UUID128_INIT(0xaa, 0xf4, 0x82, 0xdd, 0x28, 0xa7, 0xac, 0x86, 0x68,
//                0x4d, 0xd5, 0x40, 0x3f, 0x11, 0xdd, 0xcc);
//
//static char rm_demo_write_data[64] = "This is the command characteristic";
//
///* GATT service definitions */
//static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
//    {
//        /* Environmental Service Sensing */
//        .type = BLE_GATT_SVC_TYPE_PRIMARY,
//        .uuid = BLE_UUID16_DECLARE(BLE_GATT_SVC_ESS),
//        .characteristics = (struct ble_gatt_chr_def[]) { 
//            {
//            /* Characteristic: Read only latest measurement */
//                .uuid = (ble_uuid_t *)&latest_mesurement_uuid.u,
//                .access_cb = send_latest_measurements,
//                .val_handle = &_hrs_val_handle,
//                .flags = BLE_GATT_CHR_F_NOTIFY,
//            },  
//        }
//    },
//    {   /* Service: Read/Write Demo */
//        .type = BLE_GATT_SVC_TYPE_PRIMARY,
//        .uuid = (ble_uuid_t *)&gatt_svr_svc_rw_demo_uuid.u,
//        .characteristics = (struct ble_gatt_chr_def[]){
//         {
//             /* Characteristic: Read/Write Demo write */
//             .uuid = (ble_uuid_t *)&gatt_svr_chr_rw_demo_write_uuid.u,
//             .access_cb = gatt_svr_chr_access_rw_demo,
//             .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
//         },
//         {
//             /* Characteristic: Read/Write Demo read only */
//             .uuid = (ble_uuid_t *)&gatt_svr_chr_rw_demo_readonly_uuid.u,
//             .access_cb = gatt_svr_chr_access_rw_demo,
//             .flags = BLE_GATT_CHR_F_READ,
//         },
//            {
//                0, /* no more characteristics in this service */
//            }, 
//        }
//    },
//    {
//        /* Device Information Service */
//        .type = BLE_GATT_SVC_TYPE_PRIMARY,
//        .uuid = BLE_UUID16_DECLARE(BLE_GATT_SVC_DEVINFO),
//        .characteristics = (struct ble_gatt_chr_def[]) { {
//            .uuid = BLE_UUID16_DECLARE(BLE_GATT_CHAR_MANUFACTURER_NAME),
//            .access_cb = _devinfo_handler,
//            .flags = BLE_GATT_CHR_F_READ,
//        }, {
//            .uuid = BLE_UUID16_DECLARE(BLE_GATT_CHAR_MODEL_NUMBER_STR),
//            .access_cb = _devinfo_handler,
//            .flags = BLE_GATT_CHR_F_READ,
//        }, {
//            .uuid = BLE_UUID16_DECLARE(BLE_GATT_CHAR_SERIAL_NUMBER_STR),
//            .access_cb = _devinfo_handler,
//            .flags = BLE_GATT_CHR_F_READ,
//        }, {
//            .uuid = BLE_UUID16_DECLARE(BLE_GATT_CHAR_FW_REV_STR),
//            .access_cb = _devinfo_handler,
//            .flags = BLE_GATT_CHR_F_READ,
//        }, {
//            .uuid = BLE_UUID16_DECLARE(BLE_GATT_CHAR_HW_REV_STR),
//            .access_cb = _devinfo_handler,
//            .flags = BLE_GATT_CHR_F_READ,
//        }, {
//            0, /* no more characteristics in this service */
//        }, }
//    },
//    {
//        0, /* no more services */
//    },
//};

//static int _devinfo_handler(uint16_t conn_handle, uint16_t attr_handle,
//                            struct ble_gatt_access_ctxt *ctxt, void *arg)
//{
//    (void)conn_handle;
//    (void)attr_handle;
//    (void)arg;
//    const char *str;
//
//    switch (ble_uuid_u16(ctxt->chr->uuid)) {
//        case BLE_GATT_CHAR_MANUFACTURER_NAME:
//            puts("[READ] device information service: manufacturer name value");
//            str = _manufacturer_name;
//            break;
//        case BLE_GATT_CHAR_MODEL_NUMBER_STR:
//            puts("[READ] device information service: model number value");
//            str = _model_number;
//            break;
//        case BLE_GATT_CHAR_SERIAL_NUMBER_STR:
//            puts("[READ] device information service: serial number value");
//            str = _serial_number;
//            break;
//        case BLE_GATT_CHAR_FW_REV_STR:
//            puts("[READ] device information service: firmware revision value");
//            str = _fw_ver;
//            break;
//        case BLE_GATT_CHAR_HW_REV_STR:
//            puts("[READ] device information service: hardware revision value");
//            str = _hw_ver;
//            break;
//        default:
//            return BLE_ATT_ERR_UNLIKELY;
//    }
//
//    int res = os_mbuf_append(ctxt->om, str, strlen(str));
//    return (res == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
//}
//
//
//static int gap_event_cb(struct ble_gap_event *event, void *arg)
//{
//    (void)arg;
//
//    switch (event->type) {
//        case BLE_GAP_EVENT_CONNECT:
//            if (event->connect.status) {
//                _stop_updating();
//                nimble_autoadv_start();
//                return 0;
//            }
//            _conn_handle = event->connect.conn_handle;
//            break;
//
//        case BLE_GAP_EVENT_DISCONNECT:
//            _stop_updating();
//            nimble_autoadv_start();
//            break;
//
//        case BLE_GAP_EVENT_SUBSCRIBE:
//            if (event->subscribe.attr_handle == _hrs_val_handle) {
//                if (event->subscribe.cur_notify == 1) {
//                    _start_updating();
//                }
//                else {
//                    _stop_updating();
//                }
//            }
//            break;
//    }
//
//    return 0;
//}
//
//static void _start_updating(void)
//{
//    event_timeout_set(&_update_timeout_evt, UPDATE_INTERVAL);
//    puts("[NOTIFY_ENABLED] environmental sensing service");
//}
//
//static void _stop_updating(void)
//{
//    event_timeout_clear(&_update_timeout_evt);
//    puts("[NOTIFY_DISABLED] environmental sensing service");
//}

//static void _hr_update(event_t *e)
//{
//    (void)e;
//    struct os_mbuf *om;
//
//    memcpy(sensor_measurements,latitude,sizeof(float));
//    memcpy(sensor_measurements + sizeof(float),longitude,sizeof(float));
//    memcpy(sensor_measurements + sizeof(float) + sizeof(float),speed,sizeof(float));
//
//
//    //scd30_read_triggered(&scd30_dev, &result);
//    //memcpy(sensor_measurements,&result.co2_concentration,sizeof(float));
//    //uint32_t pressure = bmx280_read_pressure(&dev);
//    //memcpy(sensor_measurements + sizeof(float), &pressure, sizeof(uint32_t));                                                   
//    //int16_t temperature = bmx280_read_temperature(&dev);
//    //memcpy(sensor_measurements + sizeof(float) + sizeof(uint32_t), &temperature, sizeof(int16_t));
//    //int humidity = bme280_read_humidity(&dev);
//    //memcpy(sensor_measurements + sizeof(float) + sizeof(uint32_t) + sizeof(int16_t), &humidity, sizeof(int));
//    //si1133_capture_sensors(&devSI, valuesSI11, ARRAY_SIZE(valuesSI11));
//    //uint32_t lum = valuesSI11[1];
//    //memcpy(sensor_measurements + sizeof(float) + sizeof(uint32_t) + sizeof(int16_t) + sizeof(int), &lum, sizeof(uint32_t));
//    //uint32_t uv = valuesSI11[2];
//    //memcpy(sensor_measurements + sizeof(float) + sizeof(uint32_t) + sizeof(int16_t) + sizeof(int) + sizeof(uint32_t), &uv, sizeof(uint32_t));
//    //int sampleADC = adc_sample(ADC_LINE(7), RES);
//    //memcpy(sensor_measurements + sizeof(float) + sizeof(uint32_t) + sizeof(int16_t) + sizeof(int) + sizeof(uint32_t) + sizeof(uint32_t), &sampleADC, sizeof(uint32_t));
//    //
//    //memcpy(sensor_measurements + sizeof(float) + sizeof(uint32_t) + sizeof(int16_t) + sizeof(int) + sizeof(uint32_t) + sizeof(uint32_t)
//    //       + sizeof(float), latitude, sizeof(float));
//    //memcpy(sensor_measurements + sizeof(float) + sizeof(uint32_t) + sizeof(int16_t) + sizeof(int) + sizeof(uint32_t) + sizeof(uint32_t)
//    //       + sizeof(float) + sizeof(float), longitude, sizeof(float));
//    //memcpy(sensor_measurements + sizeof(float) + sizeof(uint32_t) + sizeof(int16_t) + sizeof(int) + sizeof(uint32_t) + sizeof(uint32_t)
//    //       + sizeof(float) + sizeof(float) + sizeof(float), speed, sizeof(float));
//    
//    /* Send heart rate data notification to GATT client */
//    om = ble_hs_mbuf_from_flat(sensor_measurements, sizeof(sensor_measurements));
//    assert(om != NULL);
//    int res = ble_gattc_notify_custom(_conn_handle, _hrs_val_handle, om);
//    assert(res == 0);
//    (void)res;
//
//    //printf("   CO2 Concentration [ppm]:  %f\n\r",result.co2_concentration);
//    //printf("   Pressure    [Pa]:      %" PRIu32 "\n\r", pressure);
//    //printf("   Temperature [°C]:      %" PRIi16 "\n\r", temperature);
//    //printf("   Humidity    [Percent]: %d\n\r", humidity);
//    //printf("   Luminosidade [lux]:    %lu\n\r", lum);
//    //printf("   UV [lux]:              %lu\n\r", uv);
//    //printf("   Battery     [V]:       %d\n\r",  sampleADC);
//    printf("   Latitude     [Dec]:       %f\n\r",  *latitude);
//    printf("   Longitude     [Dec]:       %f\n\r",  *longitude);
//    printf("   Speed     [m/s]:       %f\n\r",  *speed);
//
//    /* Schedule next update event */
//    event_timeout_set(&_update_timeout_evt, UPDATE_INTERVAL);
//
//}

//static int gatt_svr_chr_access_rw_demo(
//    uint16_t conn_handle, uint16_t attr_handle,
//    struct ble_gatt_access_ctxt *ctxt, void *arg)
//{
//    puts("service 'rw demo' callback triggered");
//
//    (void)conn_handle;
//    (void)attr_handle;
//    (void)arg;
//
//    int rc = 0;
//
//    ble_uuid_t *write_uuid = (ble_uuid_t *)&gatt_svr_chr_rw_demo_write_uuid.u;
//    ble_uuid_t *readonly_uuid = (ble_uuid_t *)&gatt_svr_chr_rw_demo_readonly_uuid.u;
//
//    if (ble_uuid_cmp(ctxt->chr->uuid, write_uuid) == 0)
//    {
//
//        puts("access to characteristic 'rw demo (write)'");
//        printf("entrou aqui\n");
//
//        switch (ctxt->op)
//        {
//
//        case BLE_GATT_ACCESS_OP_READ_CHR:
//            puts("read from characteristic");
//            printf("current value of rm_demo_write_data: '%s'\n \r",
//                   rm_demo_write_data);
//
//            /* Send given data to the client */
//            rc = os_mbuf_append(ctxt->om, &rm_demo_write_data,
//                                strlen(rm_demo_write_data));
//
//            break;
//
//        case BLE_GATT_ACCESS_OP_WRITE_CHR:
//            puts("write to characteristic");
//
//            printf("old value of rm_demo_write_data: '%s'\n \r",
//                   rm_demo_write_data);
//
//            uint16_t om_len;
//            om_len = OS_MBUF_PKTLEN(ctxt->om);
//
//            /* Read sent data */
//            rc = ble_hs_mbuf_to_flat(ctxt->om, &rm_demo_write_data,
//                                     sizeof rm_demo_write_data, &om_len);
//            /* We need to null-terminate the received string */
//            rm_demo_write_data[om_len] = '\0';
//
//            printf("new value of rm_demo_write_data: '%s'\n \r",
//                   rm_demo_write_data);
//
//            break;
//
//        case BLE_GATT_ACCESS_OP_READ_DSC:
//            puts("read from descriptor");
//            break;
//
//        case BLE_GATT_ACCESS_OP_WRITE_DSC:
//            puts("write to descriptor");
//            break;
//
//        default:
//            puts("unhandled operation!");
//            rc = 1;
//            break;
//        }
//
//        puts("");
//
//        return rc;
//    }
//    else if (ble_uuid_cmp(ctxt->chr->uuid, readonly_uuid) == 0)
//    {
//        puts("access to characteristic 'rw demo (read-only)'");
//
//        printf("%d\n", ctxt->op);
//
//        if(ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR){
//            printf("AAA\n");
//        }
//
//        printf("rm_demo_write_data: %s\n", rm_demo_write_data);
//
//        if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR && strcmp(rm_demo_write_data, "oi") == 0)  // Eh aqui que envia pelo Lora
//        {                                                                                      // Tirar daqi de dentro e colocar na main
//
//            sender_pid = thread_create(sender_stack, sizeof(sender_stack),
//                               SENDER_PRIO, 0, sender, NULL, "sender");
//
//            msg_t msg;
//
//            msg_send(&msg, sender_pid); 
//            
//            return rc;
//        }
//        else if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR && strcmp(rm_demo_write_data, "ri") == 0)
//        {
//            printf("entrou na ri\n\r"); //Que print eh esse?
//            sender_pid = KERNEL_PID_UNDEF;
//        }
//
//        return 0;
//    }
//
//    puts("unhandled uuid!");
//    return 1;
//}

//#define MEASUREMENT_INTERVAL_SECS (2)
//#define TEST_ITERATIONS 3


// Implementacao na raca do ringbuffer p/ lora
struct gps_data {
    float latitude;
    float longitude;
    float speed;
    float timestamp;
};
struct gps_data *gps;

struct lora_ringbuffer{
    int size;
    int pos;
    struct gps_data queue[LORAWAN_BUFSIZE];
};
struct lora_ringbuffer *lora_rb;

/* Init lorawan ringbuffer */
void init_lora_ringbuffer(struct lora_ringbuffer *ringbuffer) {
    ringbuffer->pos = 0;
    ringbuffer->size = 0;
}

/* Adds the gps data into the last position of ring ringbuffer. */
/* It overwrites the data if the ringbuffer is full.            */
void lora_ringbuffer_push(struct lora_ringbuffer *ring, float latitude, float longitude) {

    printf("CHECAGEM DENTRO DO PUSH0: %f, %f\n", latitude, longitude);

    (&(ring->queue[ring->pos]))->latitude = latitude;
    (&(ring->queue[ring->pos]))->longitude = longitude;
    
    //ring->queue[ring->pos] = *data;

    
    printf("CHECAGEM DENTRO DO PUSH: %f, %f\n", ring->queue[ring->pos].latitude, ring->queue[ring->pos].longitude);
    printf("CHECAGEM DENTRO DO PUSH2: %f, %f\n", (&(ring->queue[ring->pos]))->latitude, (&(ring->queue[ring->pos]))->longitude);

    if (ring->pos >= LORAWAN_BUFSIZE)
        ring->pos = 0;
    else
        ring->pos++;

    if (ring->size < LORAWAN_BUFSIZE)
        ring->size++;
}

/* Returns the last element of ring in data.               */
/* Returns -1 if ring is empty or 0 if sucessfully added.  */
int lora_ringbuffer_pop(struct lora_ringbuffer *ring, struct gps_data *data)
{
    if (ring->size == 0)
        return -1; 
    else
        ring->size--;

    if (ring->pos == 0)
        ring->pos = LORAWAN_BUFSIZE-1;
    else
        ring->pos--;
    
    *data = ring->queue[ring->pos];

    return 0; 
}



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


   //int16_t temperature = bmx280_read_temperature(&dev);
   //uint32_t pressure = bmx280_read_pressure(&dev);
   //int humidity = bme280_read_humidity(&dev);
   //scd30_read_triggered(&scd30_dev, &result);
   //si1133_capture_sensors(&devSI, valuesSI11, ARRAY_SIZE(valuesSI11));
   //int sampleADC = adc_sample(ADC_LINE(7), RES);

   /* Allocates memory for the characters array (char*) with  +1 for null character \0 */
   char* char_array = (char*)malloc(70*sizeof(char));
   //result.co2_concentration *= 100;
   struct gps_data *readings = NULL;

   

   
    if(!(lora_ringbuffer_pop(lora_rb, readings))) 
        puts("Couldn't get gps readings!");
    

    printf("DADOS: %f, %f: \n", readings->latitude, readings->longitude);

    if(isnan(readings->latitude) && isnan(readings->longitude)) {
        readings->latitude = 0;
        readings->longitude = 0;
    }

    printf("DADOS 2: %f, %f: \n", readings->latitude, readings->longitude);

    snprintf(char_array,77,"%f;%f", readings->latitude, readings->longitude);


   printf("%u\n", sizeof(&char_array));
   printf("%s\n", char_array);
   printf("%u\n", sizeof(char));
   printf("%u\n", sizeof(float));
   printf("%u", sizeof(double));
   
   ///* Keeps the negative signal (-) at the char_array beggining when temperature is negative */ 
   //if (temperature < 0) {
   //    memmove(char_array + 1, char_array, 6);
   //    char_array[0] = '-';
   //}

   /* Destination vector (char*) */ 
   char* destination = (char*)malloc(70*sizeof(char));
   
   /* Copies char_array to destination */ 
   strcpy(destination, char_array);


   /* Try to send the message */
   uint8_t ret = semtech_loramac_send(&loramac,(uint8_t *)destination, strlen(destination));

   //printf("Buffer conteudo: %s\n", ctx_lora.rx_buf.buf);

   //uint8_t ret = semtech_loramac_send(&loramac,(uint8_t *)(&ctx_lora.rx_buf.buf), strlen(ctx_lora.rx_buf.buf));

    //printf("Size destination: %u\n", sizeof(destination));
    //printf("Destination: %s\n", destination);

   if (ret != SEMTECH_LORAMAC_TX_DONE)  {
       printf("Cannot send message '%s', ret code: %d\n\n", message, ret);
       return;
   }

   free(char_array);
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

static void *gps_handler(void *arg)
{
    (void)arg;
    msg_t msg;
    msg_t msg_queue[8];
    msg_init_queue(msg_queue, 8);

    //float lat, lon, sped;

    int pos = 0;
    char line[MINMEA_MAX_LENGTH];

    while (1) {
        msg_receive(&msg);
        char c;

        do {
            c = (char)ringbuffer_get_one(&(ctx.rx_buf));
            if (c == '\n') {
                line[pos++] = c;
		        pos = 0;
                switch (minmea_sentence_id(line, false)) {
                    case MINMEA_SENTENCE_RMC: {
                        struct minmea_sentence_rmc frame;
                        if (minmea_parse_rmc(&frame, line)) {

                            
                            

                            printf("$RMC floating point degree coordinates and speed: (%f,%f) %f\n",
                                    minmea_tocoord(&frame.latitude),
                                    minmea_tocoord(&frame.longitude),
                                    minmea_tofloat(&frame.speed));


                                    //lat = minmea_tocoord(&frame.latitude);
                                    //lon = minmea_tocoord(&frame.longitude);
                                    //sped = minmea_tofloat(&frame.speed);

                                    gps->latitude = minmea_tocoord(&frame.latitude);
                                    gps->longitude = minmea_tocoord(&frame.longitude);
                                    gps->speed = minmea_tofloat(&frame.speed);

                                    printf("CHECAGEM DADOS0: %f, %f: \n", gps->latitude, gps->longitude);


                                    lora_ringbuffer_push(lora_rb, minmea_tocoord(&frame.latitude), minmea_tocoord(&frame.longitude));

                                    printf("CHECAGEM DADOS1: %f, %f: \n", gps->latitude, gps->longitude);

                                    printf("CHECAGEM PUSH: %f, %f: \n",lora_rb->queue[0].latitude, lora_rb->queue[0].longitude);
                                    
                                    
                                    
                                    //latitude = &lat;
                                    //longitude = &lon;
                                    //speed = &sped;

                                    //ringbuffer_add_one(&(ctx_lora.rx_buf), (char)(lat));
                                    //ringbuffer_add_one(&(ctx_lora.rx_buf), (char)(lon));
                                    //ringbuffer_add_one(&(ctx_lora.rx_buf), (char)(sped));


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
                    } break;
                    default: break;
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


int main(void)
{

    puts("\nRIOT ADC peripheral driver test\n");
    puts("This test will sample all available ADC lines once every 100ms with\n"
         "a 10-bit resolution and print the sampled results to STDIO\n\n");

    // LED //// Descomentar abaixo para ativar o shell

    //kernel_pid_t led_pid;
    //run_blink_led(&led_pid);
    //
    //char line_buf[SHELL_DEFAULT_BUFSIZE];
    //shell_run(NULL, line_buf, SHELL_DEFAULT_BUFSIZE);
  


    //////// FIM LED
    
    
    
    
    /* Initialize gps ringbuffer */ //TIRAR DEPOIS?
    ringbuffer_init(&(ctx.rx_buf), ctx.rx_mem, UART_BUFSIZE);

    /* Initialize lora ringbuffer */
    init_lora_ringbuffer(lora_rb);


    init_gps();
    
    /* Start the gps_handler thread */
    gps_handler_pid = thread_create(gps_handler_stack, sizeof(gps_handler_stack), GPS_HANDLER_PRIO, 0, gps_handler, NULL, "gps_handler");

    

    


    //ringbuffer_init(&(ctx_lora.rx_buf), ctx_lora.rx_mem, LORAWAN_BUFSIZE);

    
    ///* Initialize all available ADC lines */
    //for (unsigned i = 0; i < ADC_NUMOF; i++) {
    //    if (adc_init(ADC_LINE(7)) < 0) {
    //        printf("Initialization of ADC_LINE(%u) failed\n", 7);
    //        return 1;
    //    } else {
    //        printf("Successfully initialized ADC_LINE(%u)\n", 7);
    //    }
    //}

    //uint32_t failures = 0;

    //puts("Testing Si1133 in blocking mode:\n\r");
    //static const si1133_params_t blocking_params = {
    //    .i2c_dev = SI1133_PARAM_I2C_DEV,
    //    .address = SI1133_PARAM_ADDR
    //};
    //EXPECT_RET_CODE(SI1133_OK, si1133_init(&devSI, &blocking_params));

    //static const si1133_sensor_t sensor_list[] = {
    //    SI1133_SENS_SMALL_IR,
    //    SI1133_SENS_MEDIUM_IR,
    //    SI1133_SENS_LARGE_IR,
    //    SI1133_SENS_WHITE,
    //    SI1133_SENS_LARGE_WHITE,
    //    SI1133_SENS_UV,
    //    SI1133_SENS_DEEP_UV,
    //};
    ///* Test reading a sample one by one. */
    //for (uint32_t i = 0; i < ARRAY_SIZE(sensor_list); i++) {
    //    EXPECT_RET_CODE(SI1133_OK,
    //                    si1133_easy_configure(&devSI, sensor_list[i], 0, 0));
    //    int32_t valueSI;
    //    EXPECT_RET_CODE(SI1133_OK,
    //                    si1133_capture_sensors(&devSI, &valueSI, 1));
    //    if (valueSI >= 0x7fffff) {
    //        printf("ERROR: Sensor sample overflow, got %" PRId32 "\n\r", valueSI);
    //        failures++;
    //    }
    //    printf(" - sensor 0x%.2x: %" PRId32 "\n\r", (int)sensor_list[i], valueSI);
    //}

    ///* Test increasing the sw_gain until we get an overflow. */
    //for (uint32_t sw_gain = 0; sw_gain <= 7; sw_gain++) {
    //    uint8_t sensor_mask =
    //        SI1133_SENS_LARGE_IR |
    //        SI1133_SENS_LARGE_WHITE |
    //        SI1133_SENS_UV;
    //    EXPECT_RET_CODE(SI1133_OK,
    //                    si1133_easy_configure(&devSI, sensor_mask, 1, sw_gain));
    //    int32_t values[3];
    //    si1133_ret_code_t ret =
    //        si1133_capture_sensors(&devSI, values, ARRAY_SIZE(values));
    //    printf("INFO: sw_gain=%" PRIu32 " LARGE_IR=%10" PRId32
    //           " LARGE_WHITE=%10" PRId32 " UV=%10" PRId32 "\n\r",
    //           sw_gain, values[0], values[1], values[2]);
    //    if (ret == SI1133_OK) {
    //        continue;
    //    }
    //    /* If we didn't get an OK we should have an overflow condition. */
    //    EXPECT_RET_CODE(SI1133_ERR_OVERFLOW, ret);
    //    /* One of the values must be in overflow state. */
    //    bool overflowed = false;
    //    for (uint32_t i = 0; i < ARRAY_SIZE(values); i++) {
    //        overflowed = overflowed || values[i] == 0x7fffff;
    //    }
    //    if (!overflowed) {
    //        printf(
    //            "ERROR: Sensor overflow but no value in overflowed state.\n\r");
    //        for (uint32_t i = 0; i < ARRAY_SIZE(values); i++) {
    //            printf("  values[%" PRIu32 "] = %" PRId32 "\n\r", i, values[i]);
    //        }
    //        failures++;
    //    }
    //    else {
    //        printf("NOTE: Overflow test OK.\n\r");
    //    }
    //}
    ///* Reading any sensor after overflowing should not fail. */
    //EXPECT_RET_CODE(SI1133_OK,
    //                si1133_easy_configure(&devSI, SI1133_SENS_SMALL_IR, 1, 0));
    //int32_t valueSI;
    //EXPECT_RET_CODE(SI1133_OK,
    //                si1133_capture_sensors(&devSI, &valueSI, 1));

    ///* Test reading most sensors at once. The maximum is 6 sensors. */
    //uint32_t all = 0;
    //for (uint32_t i = 0; i < ARRAY_SIZE(sensor_list); i++) {
    //    all |= sensor_list[i];
    //}
    //EXPECT_RET_CODE(SI1133_ERR_PARAMS, si1133_easy_configure(&devSI, all, 1, 0));

    /* All except one is lower than the limit of 6. */
  
    //int32_t values[6];

    //uint32_t sw_gain=2;
    //uint8_t sensor_mask =
    //        SI1133_SENS_LARGE_IR |
    //        SI1133_SENS_LARGE_WHITE |
    //        SI1133_SENS_UV;
    //    EXPECT_RET_CODE(SI1133_OK,
    //                    si1133_easy_configure(&devSI, sensor_mask, 1, sw_gain));

    //for(uint32_t i=0; i<10; i++){
    //    EXPECT_RET_CODE(SI1133_OK,
    //                si1133_capture_sensors(&devSI, values, ARRAY_SIZE(values)));
    //    printf("INFO: sw_gain=%" PRIu32 " LARGE_IR=%10" PRId32
    //           " LARGE_WHITE=%10" PRId32 " UV=%10" PRId32 "\n\r",
    //           sw_gain, values[0], values[1], values[2]);
    //    xtimer_sleep(1);
    //}

    //if (failures != 0) {
    //    printf("Result: FAILED %" PRIu32 "\n\r", failures);
    //}
    //else {
    //    puts("Result: OK\n\r");
    //}

    //printf("SCD30 Test:\n\r");
    //int i = 0;

    //scd30_init(&scd30_dev, &params);
    //uint16_t pressure_compensation = SCD30_DEF_PRESSURE;
    //uint16_t value = 0;
    //uint16_t interval = MEASUREMENT_INTERVAL_SECS;

    //scd30_set_param(&scd30_dev, SCD30_INTERVAL, interval);
    //scd30_set_param(&scd30_dev, SCD30_START, pressure_compensation);

    //scd30_get_param(&scd30_dev, SCD30_INTERVAL, &value);
    //printf("[test][dev-%d] Interval: %u s\n\r", i, value);
    //scd30_get_param(&scd30_dev, SCD30_T_OFFSET, &value);
    //printf("[test][dev-%d] Temperature Offset: %u.%02u C\n\r", i, value / 100u,
    //       value % 100u);
    //scd30_get_param(&scd30_dev, SCD30_A_OFFSET, &value);
    //printf("[test][dev-%d] Altitude Compensation: %u m\n\r", i, value);
    //scd30_get_param(&scd30_dev, SCD30_ASC, &value);
    //printf("[test][dev-%d] ASC: %u\n\r", i, value);
    //scd30_get_param(&scd30_dev, SCD30_FRC, &value);
    //printf("[test][dev-%d] FRC: %u ppm\n\r", i, value);
    //puts("NimBLE Heart Rate Sensor Example");
    //
    //while (i < TEST_ITERATIONS) {
    //    xtimer_sleep(1);
    //    scd30_read_triggered(&scd30_dev, &result);
    //    printf(
    //        "[scd30_test-%d] Triggered measurements co2: %.02fppm,"
    //        " temp: %.02f°C, hum: %.02f%%. \n\r", i, result.co2_concentration,
    //        result.temperature, result.relative_humidity);
    //    i++;
    //}

    //i = 0;
    //scd30_start_periodic_measurement(&scd30_dev, &interval,
    //                                 &pressure_compensation);

    //while (i < TEST_ITERATIONS) {
    //    xtimer_sleep(MEASUREMENT_INTERVAL_SECS);
    //    scd30_read_periodic(&scd30_dev, &result);
    //    printf(
    //        "[scd30_test-%d] Continuous measurements co2: %.02fppm,"
    //        " temp: %.02f°C, hum: %.02f%%. \n\r", i, result.co2_concentration,
    //        result.temperature, result.relative_humidity);
    //    i++;
    //}

    //printf("BME280 Test\n\r");

    //switch (bmx280_init(&dev, &bmx280_params[0])) {
    //    case BMX280_ERR_BUS:
    //        puts("[Error] Something went wrong when using the I2C bus");
    //        break;
    //    case BMX280_ERR_NODEV:
    //        puts("[Error] Unable to communicate with any BMX280 device");
    //        break;
    //    default:
    //        /* All good -> do nothing */
    //        printf("BME280 initialized\n");
    //        break;
    //}

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

    sender_pid = thread_create(sender_stack, sizeof(sender_stack),
                               SENDER_PRIO, 0, sender, NULL, "sender");

    msg_t msg_s;
    msg_send(&msg_s, sender_pid); 


    //printf("Nimble GATT application\n\r");

    //int res = 0;
    //(void)res;

    ///* Setup local event queue (for handling heart rate updates) */
    //event_queue_init(&_eq);
    //_update_evt.handler = _hr_update;
    //event_timeout_ztimer_init(&_update_timeout_evt, ZTIMER_MSEC, &_eq, &_update_evt);

    ///* Verify and add our custom services */
    //res = ble_gatts_count_cfg(gatt_svr_svcs);
    //assert(res == 0);
    //res = ble_gatts_add_svcs(gatt_svr_svcs);
    //assert(res == 0);

    ///* Set the device name */
    //ble_svc_gap_device_name_set(NIMBLE_AUTOADV_DEVICE_NAME);
    ///* Reload the GATT server to link our added services */
    //ble_gatts_start();

    //struct ble_gap_adv_params advp;
    //memset(&advp, 0, sizeof(advp));

    //advp.conn_mode = BLE_GAP_CONN_MODE_UND;
    //advp.disc_mode = BLE_GAP_DISC_MODE_GEN;
    //advp.itvl_min  = BLE_GAP_ADV_FAST_INTERVAL1_MIN;
    //
    //advp.itvl_max  = BLE_GAP_ADV_FAST_INTERVAL1_MAX;

    /* Set advertise params */
    //nimble_autoadv_set_ble_gap_adv_params(&advp);

    /* Configure and set the advertising data */
    //nimble_autoadv_add_field(BLE_GAP_AD_UUID16_INCOMP, &latest_mesurement_uuid, sizeof(latest_mesurement_uuid));

    //nimble_auto_adv_set_gap_cb(&gap_event_cb, NULL);

    /* Start to advertise this node */
    //nimble_autoadv_start();

    /* Run an event loop for handling the heart rate update events */
    //event_loop(&_eq);

    return 0;
}