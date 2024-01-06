#ifndef SWITCH_COMMANDS_H
#define SWITCH_COMMANDS_H

#include "hoja_includes.h"

#define SW_OUT_ID_RUMBLE_CMD 0x01
#define SW_OUT_ID_RUMBLE 0x10

#define SW_CMD_GET_STATE        0x00
#define SW_CMD_SET_PAIRING      0x01
#define SW_CMD_GET_DEVICEINFO   0x02
#define SW_CMD_SET_INPUTMODE    0x03
#define SW_CMD_GET_TRIGGERET    0x04
#define SW_CMD_GET_PAGELIST     0x05
#define SW_CMD_SET_HCI          0x06
#define SW_CMD_SET_SHIPMODE     0x08
#define SW_CMD_GET_SPI          0x10
#define SW_CMD_SET_SPI          0x11
#define SW_CMD_SET_NFC          0x21
#define SW_CMD_SET_NFC_STATE    0x22
#define SW_CMD_ENABLE_IMU       0x40
#define SW_CMD_SET_IMUSENS      0x41
#define SW_CMD_ENABLE_VIBRATE   0x48
#define SW_CMD_SET_PLAYER       0x30
#define SW_CMD_GET_PLAYER       0x31
#define SW_CMD_33               0x33

void switch_rumble_translate(const uint8_t *data);
void ns_report_handler(uint8_t report_id, uint8_t *data, uint16_t len);

void ns_report_clear(uint8_t *buffer, uint16_t size);
void ns_report_setack(uint8_t ack);
void ns_report_setsubcmd(uint8_t *buffer, uint8_t command);
void ns_report_settimer(uint8_t *buffer);
void ns_report_setbattconn(uint8_t *buffer);
void ns_report_sub_setdevinfo(uint8_t *buffer);
void ns_report_sub_triggertime(uint8_t *buffer, uint16_t time_10_ms);
void ns_report_setinputreport_full(uint8_t *buffer, sw_input_s *input_data);
void ns_report_bulkset(uint8_t *buffer, uint8_t start_idx, uint8_t* data, uint8_t len);
void _ns_report_setinputreport_short(uint8_t *buffer, sw_input_s *input_data);

#endif
