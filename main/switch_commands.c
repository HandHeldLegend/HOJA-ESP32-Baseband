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
  static int16_t _switch_timer = 0;
  buffer[0] = (uint8_t)_switch_timer;
  // printf("Td=%d \n", _switch_timer);
  _switch_timer += 1;
  if (_switch_timer > 0xFF)
  {
    _switch_timer -= 0xFF;
  }
}

void ns_report_setbattconn(uint8_t *buffer)
{
  typedef union
  {
    struct
    {
      uint8_t connection : 4;
      uint8_t bat_lvl : 4;
    };
    uint8_t bat_status;
  } bat_status_u;

  bat_status_u s = {
      .bat_lvl = 8,
      .connection = 0};
  // Always set to USB connected
  buffer[1] = s.bat_status;
}

void ns_report_sub_setdevinfo(uint8_t *buffer)
{
  /* New firmware causes issue with gyro needs more research
  _switch_input_buffer[14] = 0x04; // NS Firmware primary   (4.x)
  _switch_input_buffer[15] = 0x33; // NS Firmware secondary (x.21) */

  buffer[14] = 0x03; // NS Firmware primary   (3.x)
  buffer[15] = 0x80; // NS Firmware secondary (x.72)

  // Procon   - 0x03, 0x02
  // N64      - 0x0C, 0x11
  // SNES     - 0x0B, 0x02
  // Famicom  - 0x07, 0x02
  // NES      - 0x09, 0x02
  // Genesis  - 0x0D, 0x02
  buffer[16] = 0x03; // Controller ID primary (Pro Controller)
  buffer[17] = 0x02; // Controller ID secondary

  /*_switch_input_buffer[18-23] = MAC ADDRESS;*/
  buffer[18] = global_loaded_settings.device_mac[0];
  buffer[19] = global_loaded_settings.device_mac[1];
  buffer[20] = global_loaded_settings.device_mac[2];
  buffer[21] = global_loaded_settings.device_mac[3];
  buffer[22] = global_loaded_settings.device_mac[4];
  buffer[23] = global_loaded_settings.device_mac[5];

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

bool shouldControllerRumble(const uint8_t *data)
{

  bool lbd = data[3] & 0x80;
  bool hbd = data[1] & 0x8;

  // Get High band amplitude
  uint8_t hba = (data[1] & 0xFE);
  uint8_t lba = (data[3] & 0x7F);

  return ((hba > 1) && !hbd) || ((lba > 0x40) && !lbd);
}

// Translate and handle rumble
void switch_rumble_translate(const uint8_t *data)
{
  if (shouldControllerRumble(data))
  {
    uint8_t upper = (data[1] & 0xFE) / 2;
    uint8_t lower = (((data[3] & 0x7F) - 0x40) * 2) + (data[2] == 0x80);
    float il = powf((float)lower / 100.0f, 2.0f);
    float iu = powf((float)upper / 100.0f, 0.5f);

    float i = (il > iu) ? il : iu;
    i = (i >= 1.0f) ? 1.0f : i;
    i*=100;
    app_set_rumble((uint8_t)i);
  }
  else
  {
    // TO DO
    app_set_rumble(0);
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
    printf("Enable IMU: %d\n", data[11]);
    // imu_set_enabled(data[11]>0);
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
    printf("Set ship mode: %X\n", data[11]);
    ns_report_setack(0x80);
    break;

  case SW_CMD_SET_HCI:
    // For now all options should shut down
    app_send_command(I2CINPUT_ID_SHUTDOWN, 0x00);
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
    printf("Set player: ");
    ns_report_setack(0x80);

    // We set pairing address here
    if (!app_compare_mac(global_loaded_settings.switch_host_mac, global_loaded_settings.paired_host_mac))
    {
      app_save_host_mac();
    }

    app_set_connected(1);

    uint8_t player = data[11] & 0xF;

    switch (player)
    {
    default:
    case 1:
      printf("1\n");
      break;

    case 3:
      printf("2\n");
      break;

    case 7:
      printf("3\n");
      break;

    case 15:
      printf("4\n");
      break;
    }
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

  // tud_hid_report(0x21, _switch_input_buffer, 64);
  esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21, _report_len, _switch_input_buffer);
}

// Handles an OUT report and responds accordingly.
void ns_report_handler(uint8_t report_id, uint8_t *data, uint16_t len)
{
  switch (report_id)
  {
  // We have command data and possibly rumble
  case SW_OUT_ID_RUMBLE_CMD:
    switch_rumble_translate(&data[1]);
    ns_subcommand_handler(data[9], data, len);
    break;

  case SW_OUT_ID_RUMBLE:
    switch_rumble_translate(&data[1]);
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

// Sets the input report for short mode.
void _ns_report_setinputreport_short(uint8_t *buffer)
{
  buffer[0] = 0x00;
  buffer[1] = 0x00;
  buffer[2] = 0x8; // ns_input_short.stick_hat;

  // To-do: Sticks
  buffer[3] = 0x00;
  buffer[4] = 0x00;
  buffer[5] = 0x00;
  buffer[6] = 0x00;
  buffer[7] = 0;
  buffer[8] = 0;
  buffer[9] = 0;
  buffer[10] = 0;
}

void ns_report_bulkset(uint8_t *buffer, uint8_t start_idx, uint8_t *data, uint8_t len)
{
  memcpy(&buffer[start_idx], data, len);
}
