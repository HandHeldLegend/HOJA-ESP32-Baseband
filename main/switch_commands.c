#include "switch_commands.h"
#include <math.h>

uint8_t _switch_input_buffer[64] = {0};
uint8_t _switch_input_report_id = 0x00;

void ns_report_clear(uint8_t *buffer, uint16_t size)
{
  memset(buffer, 0, size);
}

void ns_report_setid(uint8_t report_id)
{
  _switch_input_report_id = report_id;
}

void ns_report_setack(uint8_t ack)
{
  _switch_input_buffer[12] = ack;
}

void ns_report_setsubcmd(uint8_t *buffer, uint8_t command)
{
  buffer[13] = command;
}

void ns_report_settimer(uint8_t *buffer)
{
  static uint64_t time;

  time = get_timestamp_ms();

  buffer[0] = (uint8_t) (time % 0xFF);
}

void ns_report_setbattconn(uint8_t *buffer)
{
  bat_status_u s = {
    .bat_lvl    = 4,
    .charging   = 0,
    .connection = 0
  };

  s.val = global_live_data.bat_status.val;

  buffer[1] = s.val;
}

void ns_report_sub_setdevinfo(uint8_t *buffer)
{
  // New firmware causes issue with gyro needs more research
  _switch_input_buffer[14] = 0x04; // NS Firmware primary   (4.x)
  _switch_input_buffer[15] = 0x33; // NS Firmware secondary (x.21)

  // buffer[14] = 0x03; // NS Firmware primary   (3.x)
  // buffer[15] = 0x80; // NS Firmware secondary (x.72)

  // Procon   - 0x03, 0x02
  // N64      - 0x0C, 0x11
  // SNES     - 0x0B, 0x02
  // Famicom  - 0x07, 0x02
  // NES      - 0x09, 0x02
  // Genesis  - 0x0D, 0x02
  buffer[16] = 0x03; // Controller ID primary (Pro Controller)
  buffer[17] = 0x02; // Controller ID secondary

  /*_switch_input_buffer[18-23] = MAC ADDRESS;*/
  buffer[18] = global_loaded_settings.device_mac_switch[0];
  buffer[19] = global_loaded_settings.device_mac_switch[1];
  buffer[20] = global_loaded_settings.device_mac_switch[2];
  buffer[21] = global_loaded_settings.device_mac_switch[3];
  buffer[22] = global_loaded_settings.device_mac_switch[4];
  buffer[23] = global_loaded_settings.device_mac_switch[5];

  buffer[24] = 0x00;
  buffer[25] = 0x02; // It's 2 now? Ok.
}

void ns_report_sub_triggertime(uint8_t *buffer, uint16_t time_10_ms)
{
  uint8_t upper_ms = 0xFF & time_10_ms;
  uint8_t lower_ms = (0xFF00 & time_10_ms) >> 8;

  // Set all button groups
  // L - 15, 16
  // R - 17, 18
  // ZL - 19, 20
  // ZR - 21, 22
  // SL - 23, 24
  // SR - 25, 26
  // Home - 27, 28

  for (uint8_t i = 0; i < 14; i += 2)
  {
    buffer[14 + i] = upper_ms;
    buffer[15 + i] = lower_ms;
  }
}

// Handles a command, always 0x21 as a response ID
void ns_subcommand_handler(uint8_t subcommand, uint8_t *data, uint16_t len)
{
  uint16_t _report_len = 15;

  // Clear report
  ns_report_clear(_switch_input_buffer, 64);
  // Set Timer
  ns_report_settimer(_switch_input_buffer);
  // Set Battery
  ns_report_setbattconn(_switch_input_buffer);

  // Fill input portion
  ns_report_setinputreport_full(_switch_input_buffer);

  // Set report ID
  // not needed it's 0x21

  // Set subcmd
  ns_report_setsubcmd(_switch_input_buffer, subcommand);

  printf("CMD: ");

  switch (subcommand)
  {
  case SW_CMD_SET_NFC:
    printf("Set NFC MCU:\n");
    ns_report_setack(0x80);
    break;

  case SW_CMD_ENABLE_IMU:
    printf("Enable IMU: %d\n", data[10]);
    ns_set_imu_mode(data[10]);
    ns_report_setack(0x80);
    break;

  case SW_CMD_SET_PAIRING:
    printf("Set pairing.\n");
    // pairing_set(data[11]);
    break;

  case SW_CMD_SET_INPUTMODE:
    printf("Input mode change: %X\n", data[10]);
    ns_report_setack(0x80);
    //ns_controller_setinputreportmode(data[10]);
    break;

  case SW_CMD_GET_DEVICEINFO:
    printf("Get device info.\n");
    _report_len += 12;
    ns_report_setack(0x82);
    ns_report_sub_setdevinfo(_switch_input_buffer);
    break;

  case SW_CMD_SET_SHIPMODE:
    printf("Set ship mode: %X\n", data[10]);
    ns_report_setack(0x80);
    break;

  case SW_CMD_SET_HCI:
    // For now all options should shut down
    printf("Set HCI %X\n", data[10]);
    switch_bt_end_task();
    app_set_power_setting(POWER_CODE_OFF); // Shut down
    break;

  case SW_CMD_GET_SPI:
    printf("Read SPI. Address: %X, %X | Len: %d\n", data[11], data[10], data[14]);
    ns_report_setack(0x90);
    sw_spi_readfromaddress(_switch_input_buffer, data[11], data[10], data[14]);
    _report_len += data[14];
    break;

  case SW_CMD_SET_SPI:
    printf("Write SPI. Address: %X, %X | Len: %d\n", data[11], data[10], data[14]);
    ns_report_setack(0x80);

    // Write IMU calibration data
    if ((data[11] == 0x80) && (data[10] == 0x26))
    {
      // for(uint16_t i = 0; i < 26; i++)
      //{
      //   global_loaded_settings.imu_calibration[i] = data[16+i];
      //   printf("0x%x, ", data[16+i]);
      //   printf("\n");
      // }
      // settings_save(false);
    }

    break;

  case SW_CMD_GET_TRIGGERET:
    printf("Get trigger ET.\n");
    ns_report_setack(0x83);
    ns_report_sub_triggertime(_switch_input_buffer, 100);
    _report_len += 14;
    break;

  case SW_CMD_ENABLE_VIBRATE:
    printf("Enable vibration.\n");
    ns_report_setack(0x80);
    break;

  case SW_CMD_SET_PLAYER:
    printf("Set player: \n");
    ns_report_setack(0x80);

    // We set pairing address here
    // if (!app_compare_mac(global_loaded_settings.switch_host_mac, global_loaded_settings.paired_host_mac))
    //{
    //  app_save_host_mac();
    //}

    uint8_t player = data[10] & 0xF;
    uint8_t set_num = 0;

    switch (player)
    {
    default:
      set_num = 1; // Always set *something*
      break;
    case 0b1:
      set_num = 1;
      break;

    case 0b11:
      set_num = 2;
      break;

    case 0b111:
      set_num = 3;
      break;

    case 0b1111:
      set_num = 4;
      break;

    case 0b1001:
      set_num = 5;
      break;
    case 0b1010:
      set_num = 6;
      break;

    case 0b1011:
      set_num = 7;
      break;
    case 0b0110:
      set_num = 8;
      break;
    }

    app_set_connected_status(set_num);
    break;

  default:
    printf("Unhandled: %X\n", subcommand);
    for (uint16_t i = 0; i < len; i++)
    {
      printf("%X, ", data[i]);
    }
    printf("\n");
    ns_report_setack(0x80);
    break;
  }
  esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21, SWITCH_BT_REPORT_SIZE, _switch_input_buffer);
}

// Handles an OUT report and responds accordingly.
void ns_report_handler(uint8_t report_id, uint8_t *data, uint16_t len)
{
  switch (report_id)
  {
  // We have command data and possibly rumble
  case SW_OUT_ID_RUMBLE_CMD:
    app_set_switch_haptic(&data[1]);
    ns_subcommand_handler(data[9], data, len);
    break;

  case SW_OUT_ID_RUMBLE:
    app_set_switch_haptic(&data[1]);
    break;

  default:
    printf("Unknown report: %X\n", report_id);
    break;
  }
}



void ns_report_bulkset(uint8_t *buffer, uint8_t start_idx, uint8_t *data, uint8_t len)
{
  memcpy(&buffer[start_idx], data, len);
}
