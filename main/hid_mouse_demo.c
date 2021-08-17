/*
 * Copyright (C) 2014 BlueKitchen GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 4. Any redistribution, use, or modification is done solely for
 *    personal benefit and not for any commercial purpose or for
 *    monetary gain.
 *
 * THIS SOFTWARE IS PROVIDED BY BLUEKITCHEN GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL MATTHIAS
 * RINGWALD OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Please inquire about commercial licensing options at
 * contact@bluekitchen-gmbh.com
 *
 */

#define BTSTACK_FILE__ "hid_mouse_demo.c"

// *****************************************************************************
/* EXAMPLE_START(hid_mouse_demo): HID Mouse Classic
 *
 * @text This HID Device example demonstrates how to implement
 * an HID keyboard. Without a HAVE_BTSTACK_STDIN, a fixed demo text is sent
 * If HAVE_BTSTACK_STDIN is defined, you can type from the terminal
 */
// *****************************************************************************


#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "driver/gpio.h"
#include "driver/timer.h"
#include "driver/rtc_io.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "btstack.h"
#include "gap.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "esp_system.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "soc/rtc.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"

#define pinA GPIO_NUM_26
#define pinB GPIO_NUM_25
#define pinGND GPIO_NUM_33 //33をGNDかわりに
#define INPUT_ASSIGN (1ULL << pinA) | (1ULL << pinB)
#define OUTPUT_ASSIGN (1ULL << pinGND)
#define ENABLE_CLASSIC

#define TIMER_G TIMER_GROUP_1
#define TIMER_T TIMER_1

uint8_t hid_service_buffer[250];
static btstack_packet_callback_registration_t hci_event_callback_registration;
static uint16_t hid_cid;

esp_timer_handle_t timer;
nvs_handle_t my_nvs_handle;
TaskHandle_t time_handle;

// from USB HID Specification 1.1, Appendix B.2
const uint8_t hid_descriptor_volume[] = {
   0x05, 0x0c,  //UsagePage(Consumer)
   0x09, 0x01, //Usage(Consumer control)
   0xA1, 0x01,  //Collection (Application)
    0x09, 0xe9, //Usage(Volume Increment)
    0x09, 0xea, //Usage(Volume Decrement)
    0x15, 0x00, //Logical Minimum(0)
    0x25, 0x01, //Logical Maximum(1)
    0x75, 0x01, //ReportSize(1)
    0x95, 0x02, //ReportCount(2)
    0x81, 0x02, //Input(Data, Variable, Abs)
   0x95, 0x01,  //Report Count (1)
   0x75, 0x06, //Report Size (6)
   0x81, 0x01, //Input (Constant) ;6 bit padding
 0xC0                    // END_COLLECTION
};

unsigned char last = 0;   //REの前回値 0 0 0 0 A B 0 0 
const uint8_t rot_table[]={ 0,1, 2, 0, 2, 0, 0, 1, 1, 0, 0, 2, 0, 2, 1,0};   //RE回転状態からディスクリプタの変換テーブル 1で増音　２で小音 0で変化なし

static void volume_send_now(void){
    unsigned char  p = (gpio_get_level(pinA)) | (gpio_get_level(pinB) << 1);   //REの今の値 0 0 0 0 0 0 A B
    uint8_t report[] = {0xa1, rot_table[last | p]}; //RE前回値と今回値の和を添字に送信内容をテーブルから持ってくる
    //printf("%d\n", rot_table[last | p] );
    last = p << 2;   //今回値で更新
    
    hid_device_send_interrupt_message(hid_cid, &report[0], sizeof(report));
}

static void IRAM_ATTR gpio_isr_handler(){
    if (!hid_cid) return;
    // trigger send
    hid_device_request_can_send_now_event(hid_cid);
    timer_set_counter_value(TIMER_G, TIMER_T, 0);
}

static void set_gpio_intterupt(void){
    gpio_set_intr_type(pinA, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(pinB, GPIO_INTR_ANYEDGE);
    gpio_isr_handler_add(pinA, gpio_isr_handler, NULL);
    gpio_isr_handler_add(pinB, gpio_isr_handler, NULL);
}

static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t * packet, uint16_t packet_size){
    UNUSED(channel);
    UNUSED(packet_size);
    switch (packet_type){
        case HCI_EVENT_PACKET:
            switch (hci_event_packet_get_type(packet)){
                case HCI_EVENT_USER_CONFIRMATION_REQUEST:
                    // ssp: inform about user confirmation request
                    break;
                case HCI_EVENT_HID_META:
                    switch (hci_event_hid_meta_get_subevent_code(packet)){                      
                        case HID_SUBEVENT_CONNECTION_OPENED:
                            if (hid_subevent_connection_opened_get_status(packet) != ERROR_CODE_SUCCESS) return;
                            printf("HID Connected\n");
                            hid_cid = hid_subevent_connection_opened_get_hid_cid(packet);
                            
                            bd_addr_t pc_addr;
                            hid_subevent_connection_opened_get_bd_addr(packet, pc_addr);//接続したpcのあどれすを取得
                            char *pc_addr_str = bd_addr_to_str(pc_addr);
                            if(nvs_set_str(my_nvs_handle, "pc_addr", pc_addr_str) == ESP_OK){
                                nvs_commit(my_nvs_handle);
                            }
                            set_gpio_intterupt();//REの割り込みを設定
                            timer_start(TIMER_G, TIMER_T);
                            break;
                        case HID_SUBEVENT_CONNECTION_CLOSED:
                            printf("HID Disconnected\n");
                            hid_cid = 0;
                            gpio_isr_handler_remove(pinA);
                            gpio_isr_handler_remove(pinB);
                            //esp_timer_delete(timer);
                            break;
                        case HID_SUBEVENT_CAN_SEND_NOW:
                            volume_send_now();
                            break;
                        default:
                            break;
                    }
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }
}

void IRAM_ATTR onTime(void *arg){
    timer_group_clr_intr_status_in_isr(TIMER_G, TIMER_T);
    ets_printf("zzz...\n");
    rtc_gpio_pullup_en(pinA);
    esp_deep_sleep_start();
}

/* @section Main Application Setup
 *
 * @text Listing MainConfiguration shows main application code.
 * To run a HID Device service you need to initialize the SDP, and to create and register HID Device record with it.
 * At the end the Bluetooth stack is started.
 */

/* LISTING_START(MainConfiguration): Setup HID Device */

int btstack_main(int argc, const char * argv[]){
    gpio_config_t in_conf = {//readピンの設定
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = INPUT_ASSIGN,
        .pull_up_en = 1
    };
    gpio_config(&in_conf);
    gpio_config_t out_conf = {//readピンの設定
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = OUTPUT_ASSIGN,
        .pull_down_en = 1
    };
    gpio_config(&in_conf);
    gpio_config(&out_conf);
    gpio_install_isr_service(0);

    rtc_gpio_deinit(pinA);//dsから復帰した時用のピンリセット
    esp_sleep_enable_ext0_wakeup(pinA, 0);//sleep復帰の指定 pinAがLOWで復帰

    timer_config_t timer_conf ={
        .alarm_en = TIMER_ALARM_EN,
        .counter_en = TIMER_PAUSE,
        .intr_type = TIMER_INTR_LEVEL,
        .counter_dir = TIMER_COUNT_UP,
        .auto_reload = TIMER_AUTORELOAD_DIS,
        .divider = 80,
    };
    timer_init(TIMER_G, TIMER_T, &timer_conf);
    timer_set_counter_value(TIMER_G, TIMER_T, 0);
    timer_set_alarm_value(TIMER_G, TIMER_T, 10000000);
    timer_enable_intr(TIMER_G, TIMER_T);
    timer_isr_register(TIMER_G, TIMER_T, onTime, NULL, 0, NULL);

    // allow to get found by inquiry
    gap_discoverable_control(1);
    // use Limited Discoverable Mode; Peripheral; Pointing Device as CoD
    gap_set_class_of_device(0x250C);
    // set local name to be identified - zeroes will be replaced by actual BD ADDR
    gap_set_local_name("BT VOL 00:00:00:00:00:00");

    // allow for role switch in general and sniff mode
    gap_set_default_link_policy_settings( LM_LINK_POLICY_ENABLE_ROLE_SWITCH | LM_LINK_POLICY_ENABLE_SNIFF_MODE );
    // allow for role switch on outgoing connections - this allow HID Host to become master when we re-connect to it
    gap_set_allow_role_switch(true);

    // L2CAP
    l2cap_init();

    // SDP Server
    sdp_init();

    memset(hid_service_buffer, 0, sizeof(hid_service_buffer));

    hid_sdp_record_t hid_params = {
        .hid_device_subclass = 0x250C,
        .hid_virtual_cable = 0,
        .hid_country_code = 0,
        .hid_remote_wake = 1, 
        .hid_reconnect_initiate = 1,
        .hid_normally_connectable = 1,
        .hid_boot_device = 0, 
        .hid_ssr_host_max_latency = 0xFFFF,
        .hid_ssr_host_min_timeout = 0xFFFF,
        .hid_supervision_timeout = 2200,
        .hid_descriptor = hid_descriptor_volume,
        .hid_descriptor_size = sizeof(hid_descriptor_volume), 
        .device_name = "BT VOLUME"
    };
    
    hid_create_sdp_record(hid_service_buffer, 0x10001, &hid_params);

    sdp_register_service(hid_service_buffer);

    // HID Device
    hid_device_init(0, sizeof(hid_descriptor_volume), hid_descriptor_volume);
    // register for HCI events
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    // register for HID
    hid_device_register_packet_handler(&packet_handler);

    // turn on!
    hci_power_control(HCI_POWER_ON);

    nvs_flash_init_partition("storage");
    nvs_open_from_partition("storage", "pc_addr_NS", NVS_READWRITE, &my_nvs_handle);

    size_t read_size;
    if((nvs_get_str(my_nvs_handle, "pc_addr",NULL, &read_size)) == ESP_OK){
        bd_addr_t pc_addr;
        char *pc_addr_str = malloc(read_size);
        nvs_get_str(my_nvs_handle, "pc_addr", pc_addr_str, &read_size);
        printf("%s\n", pc_addr_str);
        sscanf_bd_addr(pc_addr_str, pc_addr);
        hid_device_connect(pc_addr, &hid_cid);
    }
    return 0;
}
/* LISTING_END */
/* EXAMPLE_END */
