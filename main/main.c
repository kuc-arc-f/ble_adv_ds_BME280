// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "bt.h"
#include "esp_log.h"
#include "esp_system.h"
#include "rom/ets_sys.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "sys/time.h"
#include "esp_deep_sleep.h"
#include "sdkconfig.h"
#include "bme280_getTask.h"
	
const int mPinRead = 4;
static const int mMaxRestart=30;
static int mRestart=0;

static const char *tag = "BLE_ADV";

static char *adv_name="D03";
//static char *adv_name="D05";

#define HCI_H4_CMD_PREAMBLE_SIZE           (4)

/*  HCI Command opcode group field(OGF) */
#define HCI_GRP_HOST_CONT_BASEBAND_CMDS    (0x03 << 10)            /* 0x0C00 */
#define HCI_GRP_BLE_CMDS                   (0x08 << 10)

#define HCI_RESET                          (0x0003 | HCI_GRP_HOST_CONT_BASEBAND_CMDS)
#define HCI_BLE_WRITE_ADV_ENABLE           (0x000A | HCI_GRP_BLE_CMDS)
#define HCI_BLE_WRITE_ADV_PARAMS           (0x0006 | HCI_GRP_BLE_CMDS)
#define HCI_BLE_WRITE_ADV_DATA             (0x0008 | HCI_GRP_BLE_CMDS)

#define HCIC_PARAM_SIZE_WRITE_ADV_ENABLE        (1)
#define HCIC_PARAM_SIZE_BLE_WRITE_ADV_PARAMS    (15)
#define HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA      (31)

#define BD_ADDR_LEN     (6)                     /* Device address length */
typedef uint8_t bd_addr_t[BD_ADDR_LEN];         /* Device address */

#define UINT16_TO_STREAM(p, u16) {*(p)++ = (uint8_t)(u16); *(p)++ = (uint8_t)((u16) >> 8);}
#define UINT8_TO_STREAM(p, u8)   {*(p)++ = (uint8_t)(u8);}
#define BDADDR_TO_STREAM(p, a)   {int ijk; for (ijk = 0; ijk < BD_ADDR_LEN;  ijk++) *(p)++ = (uint8_t) a[BD_ADDR_LEN - 1 - ijk];}
#define ARRAY_TO_STREAM(p, a, len) {int ijk; for (ijk = 0; ijk < len;        ijk++) *(p)++ = (uint8_t) a[ijk];}

enum {
    H4_TYPE_COMMAND = 1,
    H4_TYPE_ACL     = 2,
    H4_TYPE_SCO     = 3,
    H4_TYPE_EVENT   = 4
};

static uint8_t hci_cmd_buf[128];
//deep-sleep
#define GPIO_INPUT_IO_TRIGGER     0  // There is the Button on GPIO 0
//#define GPIO_DEEP_SLEEP_DURATION     60  // sleep XX seconds and then wake up
#define GPIO_DEEP_SLEEP_DURATION       300  // sleep XX seconds and then wake up
RTC_DATA_ATTR static time_t last;        // remember last boot in RTC Memory

static void execDeepSleep(){
	struct timeval now;

	printf("start Deep Sleep\n");
	gettimeofday(&now, NULL);
	printf("deep sleep (%lds since last reset, %lds since last boot)\n",now.tv_sec,now.tv_sec-last);
	last = now.tv_sec;

	printf("config Timer\n");
	esp_deep_sleep_enable_timer_wakeup(1000000LL * GPIO_DEEP_SLEEP_DURATION); // set timer but don't sleep now

	printf("config IO\n");
	esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_AUTO); //!< Keep power domain enabled in deep sleep, if it is needed by one of the wakeup options. Otherwise power it down.
	gpio_pullup_en(GPIO_INPUT_IO_TRIGGER);		// use pullup on GPIO
	gpio_pulldown_dis(GPIO_INPUT_IO_TRIGGER);       // not use pulldown on GPIO

	esp_deep_sleep_enable_ext0_wakeup(GPIO_INPUT_IO_TRIGGER, 0); // Wake if GPIO is low

	printf("deep sleep #Start#\n");
	esp_deep_sleep_start();	
}

/*
 * @brief: BT controller callback function, used to notify the upper layer that
 *         controller is ready to receive command
 */
static void controller_rcv_pkt_ready(void)
{
    printf("controller rcv pkt ready\n");
}

/*
 * @brief: BT controller callback function, to transfer data packet to upper
 *         controller is ready to receive command
 */
static int host_rcv_pkt(uint8_t *data, uint16_t len)
{
    printf("host rcv pkt: ");
    for (uint16_t i = 0; i < len; i++) {
        printf("%02x", data[i]);
    }
    printf("\n");
    return 0;
}

static esp_vhci_host_callback_t vhci_host_cb = {
    controller_rcv_pkt_ready,
    host_rcv_pkt
};

static uint16_t make_cmd_reset(uint8_t *buf)
{
    UINT8_TO_STREAM (buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM (buf, HCI_RESET);
    UINT8_TO_STREAM (buf, 0);
    return HCI_H4_CMD_PREAMBLE_SIZE;
}

static uint16_t make_cmd_ble_set_adv_enable (uint8_t *buf, uint8_t adv_enable)
{
    UINT8_TO_STREAM (buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM (buf, HCI_BLE_WRITE_ADV_ENABLE);
    UINT8_TO_STREAM  (buf, HCIC_PARAM_SIZE_WRITE_ADV_ENABLE);
    UINT8_TO_STREAM (buf, adv_enable);
    return HCI_H4_CMD_PREAMBLE_SIZE + HCIC_PARAM_SIZE_WRITE_ADV_ENABLE;
}

static uint16_t make_cmd_ble_set_adv_param (uint8_t *buf, uint16_t adv_int_min, uint16_t adv_int_max,
        uint8_t adv_type, uint8_t addr_type_own,
        uint8_t addr_type_dir, bd_addr_t direct_bda,
        uint8_t channel_map, uint8_t adv_filter_policy)
{
    UINT8_TO_STREAM (buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM (buf, HCI_BLE_WRITE_ADV_PARAMS);
    UINT8_TO_STREAM  (buf, HCIC_PARAM_SIZE_BLE_WRITE_ADV_PARAMS );

    UINT16_TO_STREAM (buf, adv_int_min);
    UINT16_TO_STREAM (buf, adv_int_max);
    UINT8_TO_STREAM (buf, adv_type);
    UINT8_TO_STREAM (buf, addr_type_own);
    UINT8_TO_STREAM (buf, addr_type_dir);
    BDADDR_TO_STREAM (buf, direct_bda);
    UINT8_TO_STREAM (buf, channel_map);
    UINT8_TO_STREAM (buf, adv_filter_policy);
    return HCI_H4_CMD_PREAMBLE_SIZE + HCIC_PARAM_SIZE_BLE_WRITE_ADV_PARAMS;
}


static uint16_t make_cmd_ble_set_adv_data(uint8_t *buf, uint8_t data_len, uint8_t *p_data)
{
    UINT8_TO_STREAM (buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM (buf, HCI_BLE_WRITE_ADV_DATA);
    UINT8_TO_STREAM  (buf, HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA + 1);

    memset(buf, 0, HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA);

    if (p_data != NULL && data_len > 0) {
        if (data_len > HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA) {
            data_len = HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA;
        }

        UINT8_TO_STREAM (buf, data_len);

        ARRAY_TO_STREAM (buf, p_data, data_len);
    }
    return HCI_H4_CMD_PREAMBLE_SIZE + HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA + 1;
}

static void hci_cmd_send_reset(void)
{
    uint16_t sz = make_cmd_reset (hci_cmd_buf);
    esp_vhci_host_send_packet(hci_cmd_buf, sz);
}

static void hci_cmd_send_ble_adv_start(void)
{
    uint16_t sz = make_cmd_ble_set_adv_enable (hci_cmd_buf, 1);
    esp_vhci_host_send_packet(hci_cmd_buf, sz);
}

static void hci_cmd_send_ble_set_adv_param(void)
{
    uint16_t adv_intv_min = 256; // 160ms
    uint16_t adv_intv_max = 256; // 160ms
    uint8_t adv_type = 0; // connectable undirected advertising (ADV_IND)
    uint8_t own_addr_type = 0; // Public Device Address
    uint8_t peer_addr_type = 0; // Public Device Address
    uint8_t peer_addr[6] = {0x80, 0x81, 0x82, 0x83, 0x84, 0x85};
    uint8_t adv_chn_map = 0x07; // 37, 38, 39
    uint8_t adv_filter_policy = 0; // Process All Conn and Scan

    uint16_t sz = make_cmd_ble_set_adv_param(hci_cmd_buf,
                  adv_intv_min,
                  adv_intv_max,
                  adv_type,
                  own_addr_type,
                  peer_addr_type,
                  peer_addr,
                  adv_chn_map,
                  adv_filter_policy);
    esp_vhci_host_send_packet(hci_cmd_buf, sz);
}

//int mCountSend=0;
//
static void hci_cmd_send_ble_set_adv_data(void)
{
    const int iDatMax=6;
    task_bme280_normal_mode(NULL);
    double bTemp =get_mBME280_Temp();
    double bHum  =get_mBME280_Hum();
    double bPress=get_mBME280_Press();
//    char sDtemp[6+1];
//    sprintf(sDtemp,  "%.1f", bTemp );
    uint8_t name_len = (uint8_t)strlen(adv_name);
    uint8_t adv_data[31] = {0x02, 0x01, 0x06, 0x0, 0x09};
    uint8_t adv_data_len;
    //data
    char str_data[6]  = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    char str_data2[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    char str_data3[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    sprintf(str_data,  "%6.1f", bTemp);
    sprintf(str_data2, "%6.1f", bHum );
    sprintf(str_data3, "%06d", (int)bPress  );
//    sprintf(str_data2, "%06d", 11);
//    sprintf(str_data3, "%06d", 22 );
    printf("#str_data=%s\n", str_data);
    
    uint8_t iDat_len = (uint8_t)strlen(str_data);
    uint8_t iDat_len2 = (uint8_t)strlen(str_data2);
    uint8_t iDat_len3 = (uint8_t)strlen(str_data3);

    adv_data[3] = (name_len+ iDat_len +iDat_len2+ iDat_len3) + 1;
    for (int i = 0; i < name_len; i++) {
        adv_data[5 + i] = (uint8_t)adv_name[i];
    }
    //d1
    for (int j = 0; j < iDatMax; j++) {
    	adv_data[5+name_len  + j] =(uint8_t )str_data[j];
    }
    //d2
    int iStpos=5+name_len +iDatMax;
    for (int k = 0; k < iDatMax ; k++) {
    	adv_data[iStpos  + k] =(uint8_t )str_data2[k];
    }
    //d3
    int iStpos2 =5+name_len + (iDatMax * 2 );
    for (int k2 = 0; k2 < iDatMax ; k2++) {
    	adv_data[iStpos2  + k2] =(uint8_t )str_data3[k2];
    }
    adv_data[26]=0x00;
    
    //debug
    printf("adv_data=%s\n",adv_data);
//    adv_data_len = 5 + name_len + (iDat_len + iDat_len2 );
    adv_data_len = 5 + name_len + (iDat_len + iDat_len2 +iDat_len3);
    uint16_t sz = make_cmd_ble_set_adv_data(hci_cmd_buf, adv_data_len, (uint8_t *)adv_data);
    esp_vhci_host_send_packet(hci_cmd_buf, sz);
    
}

/*
 * @brief: send HCI commands to perform BLE advertising;
 */
void bleAdvtTask(void *pvParameters)
{
    int cmd_cnt = 0;
    bool send_avail = false;
    esp_vhci_host_register_callback(&vhci_host_cb);
    printf("BLE advt task start\n");
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        send_avail = esp_vhci_host_check_send_available();
        if (send_avail) {
            switch (cmd_cnt) {
            case 0: hci_cmd_send_reset(); ++cmd_cnt; break;
            case 1: hci_cmd_send_ble_set_adv_param(); ++cmd_cnt; break;
            case 2: hci_cmd_send_ble_set_adv_data(); ++cmd_cnt; break;
            case 3: hci_cmd_send_ble_adv_start(); ++cmd_cnt; break;
            }
        }
        //add-reSend
        if(cmd_cnt >3){ cmd_cnt=0;  }
        printf("BLE Advertise, flag_send_avail: %d, cmd_sent: %d mRestart=%d\n", send_avail, cmd_cnt, mRestart);
        if(mRestart >= mMaxRestart ){
        	execDeepSleep();
        }
        mRestart = mRestart+1;
    }
}

void app_main()
{
	//BME280
	i2c_master_init();
    // task_bme280_normal_mode(NULL);
	//BLE
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if (esp_bt_controller_init(&bt_cfg) != ESP_OK) {
        ESP_LOGI(tag, "Bluetooth controller initialize failed");
        return;
    }

    if (esp_bt_controller_enable(ESP_BT_MODE_BTDM) != ESP_OK) {
        ESP_LOGI(tag, "Bluetooth controller enable failed");
        return;
    }

    xTaskCreatePinnedToCore(&bleAdvtTask, "bleAdvtTask", 2048, NULL, 5, NULL, 0);
}

