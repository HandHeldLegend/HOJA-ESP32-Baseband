#include "switch_commands.h"
#include <math.h>

uint8_t _switch_input_buffer[64] = {0};
uint8_t _switch_input_report_id = 0x00;
uint8_t _switch_imu_mode = 0x00;

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

uint32_t adapter_ll_get_timestamp_us()
{
  int64_t t = esp_timer_get_time();

  if (t > 0xFFFFFFFF)
    t -= 0xFFFFFFFF;
  return (uint32_t)t;
}

void ns_report_settimer(uint8_t *buffer)
{
  static uint32_t last_time;
  uint32_t time;
  static uint32_t accumulated_delta = 0;

  static uint32_t output_time = 0;

  time = get_timestamp_us();
  uint32_t delta = 0;
  uint32_t whole = 0;
  // Increment only by changed time
  if (time < last_time)
  {
    delta = (0xFFFFFFFF - last_time) + time;
  }
  else if (time >= last_time)
  {
    delta = time - last_time;
  }

  last_time = time;

  if(delta > 1000)
  {
    whole = delta;
    delta %= 1000;
    whole -= delta;
    whole /= 1000; // Convert to ms
    // Increment for the next cycle
    output_time += whole;
    output_time %= 0xFF;
  }
  else{
    output_time += 1;
    output_time %= 0xFF;
  }

  buffer[0] = (uint8_t)output_time;
}

void ns_report_setbattconn(uint8_t *buffer)
{
  bat_status_u s = {
    .bat_lvl    = 4,
    .charging   = 0,
    .connection = 0
  };

  s.val = global_live_data.bat_status.val;

  // Always set to USB connected
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
  ns_report_setinputreport_full(_switch_input_buffer, &_switch_input_data);

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
    _switch_imu_mode = (data[10]);
    ns_report_setack(0x80);
    break;

  case SW_CMD_SET_PAIRING:
    printf("Set pairing.\n");
    // pairing_set(data[11]);
    break;

  case SW_CMD_SET_INPUTMODE:
    printf("Input mode change: %X\n", data[10]);
    ns_report_setack(0x80);
    ns_controller_setinputreportmode(data[10]);
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

  ns_reset_report_spacer();
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

void ns_report_setinputreport_full(uint8_t *buffer, sw_input_s *input_data)
{

  // Set input data
  buffer[2] = input_data->right_buttons;
  buffer[3] = input_data->shared_buttons;
  buffer[4] = input_data->left_buttons;

  // Set sticks directly
  // Saves cycles :)
  buffer[5] = (input_data->ls_x & 0xFF);
  buffer[6] = (input_data->ls_x & 0xF00) >> 8;
  // ns_input_report[7] |= (g_stick_data.lsy & 0xF) << 4;
  buffer[7] = (input_data->ls_y & 0xFF0) >> 4;
  buffer[8] = (input_data->rs_x & 0xFF);
  buffer[9] = (input_data->rs_x & 0xF00) >> 8;
  buffer[10] = (input_data->rs_y & 0xFF0) >> 4;

  if (_switch_imu_mode == 0x01)
  {
    // Set gyro
    // Retrieve and write IMU data
    imu_data_s *_imu_tmp = imu_fifo_last();

    // Group 1
    buffer[12] = _imu_tmp->ay_8l; // Y-axis
    buffer[13] = _imu_tmp->ay_8h;
    buffer[14] = _imu_tmp->ax_8l; // X-axis
    buffer[15] = _imu_tmp->ax_8h;
    buffer[16] = _imu_tmp->az_8l; // Z-axis
    buffer[17] = _imu_tmp->az_8h;

    buffer[18] = _imu_tmp->gy_8l;
    buffer[19] = _imu_tmp->gy_8h;
    buffer[20] = _imu_tmp->gx_8l;
    buffer[21] = _imu_tmp->gx_8h;
    buffer[22] = _imu_tmp->gz_8l;
    buffer[23] = _imu_tmp->gz_8h;

    _imu_tmp = imu_fifo_last();

    // Group 2
    buffer[24] = _imu_tmp->ay_8l; // Y-axis
    buffer[25] = _imu_tmp->ay_8h;
    buffer[26] = _imu_tmp->ax_8l; // X-axis
    buffer[27] = _imu_tmp->ax_8h;
    buffer[28] = _imu_tmp->az_8l; // Z-axis
    buffer[29] = _imu_tmp->az_8h;

    buffer[30] = _imu_tmp->gy_8l;
    buffer[31] = _imu_tmp->gy_8h;
    buffer[32] = _imu_tmp->gx_8l;
    buffer[33] = _imu_tmp->gx_8h;
    buffer[34] = _imu_tmp->gz_8l;
    buffer[35] = _imu_tmp->gz_8h;

    _imu_tmp = imu_fifo_last();

    // Group 3
    buffer[36] = _imu_tmp->ay_8l; // Y-axis
    buffer[37] = _imu_tmp->ay_8h;
    buffer[38] = _imu_tmp->ax_8l; // X-axis
    buffer[39] = _imu_tmp->ax_8h;
    buffer[40] = _imu_tmp->az_8l; // Z-axis
    buffer[41] = _imu_tmp->az_8h;

    buffer[42] = _imu_tmp->gy_8l;
    buffer[43] = _imu_tmp->gy_8h;
    buffer[44] = _imu_tmp->gx_8l;
    buffer[45] = _imu_tmp->gx_8h;
    buffer[46] = _imu_tmp->gz_8l;
    buffer[47] = _imu_tmp->gz_8h;
  }
  else if (_switch_imu_mode == 0x02)
  {
    static mode_2_s mode_2_data = {0};

    imu_pack_quat(&mode_2_data);

    memcpy(&(buffer[12]), &mode_2_data, sizeof(mode_2_s));
  }
}

typedef union
{
  struct
  {
    uint8_t down : 1;
    uint8_t right : 1;
    uint8_t left : 1;
    uint8_t up : 1;
    uint8_t sl : 1;
    uint8_t sr : 1;
    uint8_t reserved : 2;
  };
  uint8_t button_status;
} short_buttons_1_u;

typedef union
{
  struct
  {
    uint8_t minus : 1;
    uint8_t plus : 1;
    uint8_t lstick : 1;
    uint8_t rstick : 1;
    uint8_t home : 1;
    uint8_t capture : 1;
    uint8_t lr : 1;
    uint8_t zlzr : 1;
  };
  uint8_t button_status;
} short_buttons_2_u;

// Sets the input report for short mode.
void _ns_report_setinputreport_short(uint8_t *buffer, sw_input_s *input_data)
{

  static short_buttons_1_u short_buttons_1;
  static short_buttons_2_u short_buttons_2;
  short_buttons_1.down = input_data->b_b;
  short_buttons_1.right = input_data->b_a;
  short_buttons_1.left = input_data->b_y;
  short_buttons_1.up = input_data->b_x;

  buffer[0] = short_buttons_1.button_status;
  buffer[1] = 0;
  buffer[2] = 0x8; // ns_input_short.stick_hat;

  // To-do: Sticks
  buffer[3] = input_data->ls_x & 0xFF;
  buffer[4] = (input_data->ls_x >> 8);
  buffer[5] = input_data->ls_y & 0xFF;
  buffer[6] = input_data->ls_y >> 8;
  buffer[7] = input_data->rs_x & 0xFF;
  buffer[8] = input_data->rs_x >> 8;
  buffer[9] = input_data->rs_y & 0xFF;
  buffer[10] = input_data->rs_y >> 8;
}

void ns_report_bulkset(uint8_t *buffer, uint8_t start_idx, uint8_t *data, uint8_t len)
{
  memcpy(&buffer[start_idx], data, len);
}
