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

// 40-625hz low frequency
// 585hz range
float _get_low_frequency(uint16_t value)
{
  value = (value > 127) ? 127 : value;
  const float scale = 4.60629f;

  if(!value) return 40.0f;
  else return ((float) value * scale) + 40.0f;
}

// 80-1250hz high frequency
// 1170hz range
float _get_high_frequency(uint16_t value)
{
  value = (value > 127) ? 127 : value;
  const float scale = 9.21259f;

  if(!value) return 80.0f;
  else return ((float) value * scale) + 80.0f;
}

float _get_amplitude(uint16_t value)
{
  value = (value > 127) ? 127 : value;
  if(!value) return 0;

  return (float) value/127.0f;
}

// Translate and handle rumble
void switch_rumble_translate(const uint8_t *data)
{
  // Extract modulation indicator in byte 3
  // v9 / result
  uint8_t upper2 = data[3] >> 6;
  int result = (upper2)>0;

  uint16_t hfcode = 0;
  uint16_t lacode = 0;
  uint16_t lfcode = 0;
  uint16_t hacode = 0;

  float fhi = 0;
  float ahi = 0;

  float flo = 0;
  float alo = 0;

  // Single wave w/ resonance
  // v14
  bool high_f_select = false;
  
  // v4
  uint8_t patternType = 0x00;

    // Handle different modulation types
    if (upper2) // If upper 2 bits exist
    {
        if (upper2 != 1) 
        {
            if (upper2 == 2) // Value is 2
            {
                // Check if lower 10 bytes of first two bytes exists
                uint16_t lower10 = (data[1]<<8) | data[0];
                if (lower10 & 0x03FF)
                {
                    patternType = 5;
                }
                else patternType = 2;
            } 
            else if (upper2 == 3) // Value is 3
            {
                patternType = 3;
            }
            
            goto LABEL_24;
        }
        
        // Upper 2 bits are unset...
        // The format of the rumble is treated differently.
        uint16_t lower12 = (data[2]<<8 | data[1]);
        
        if (!(lower12 & 0x0FFF)) // if lower 12 bits are empty
        {
            patternType = 1;
            goto LABEL_24;
        }
        
        // Lower 12 bits are set
        // Format is different again
        
        // Check lower 2 bits of byte 0
        // v10
        uint16_t lower2 = data[0] & 0x3;
        
        if (!lower2) // Lower 2 is blank (0x00)
        {
            patternType = 4;
            goto LABEL_24;
        }
        if ((lower2 & 2) != 0) // Lower 2, bit 1 is set (0x02 or 0x03)
        {
            result = 3;
            patternType = 7;
            goto LABEL_24;
        }
        
        // Lower 2, bit 0 is set only (0x01)
        // Do nothing?
        return;
    }
    
    // Check if there is no int value present in the data
    
    //if (!(4 * *(int *)data)) {
    //    return;
    //}
    //printf("Int found\n");

    result = 3;
    patternType = 6;

  LABEL_24:

  // Unpack codes based on modulation type
  switch (patternType) {
      case 0:
      case 1:
      case 2:
      case 3:
          //printf("Case 0-3\n");
          /*
          amFmCodes[0] = (v9 >> 1) & 0x1F;
          amFmCodes[1] = (*((unsigned short *)data + 1) >> 4) & 0x1F;
          amFmCodes[2] = (*(unsigned short *)(data + 1) >> 7) & 0x1F;
          amFmCodes[3] = (data[1] >> 2) & 0x1F;
          amFmCodes[4] = (*(unsigned short *)data >> 5) & 0x1F;
          v12 = *data & 0x1F;
          */
          return;
          break;

      // Dual frequency mode
      case 4:
          //printf("Case 4\n");
          
          // Low channel
          lfcode = (data[2]&0x7F); // Low Frequency

          lacode = ((data[3]&0x3F)<<1) | ((data[2]&0x80)>>7); // Low Amplitude
          
          // 40-625hz
          //printf("LF : %x\n", lfcode);
          //printf("LA : %x\n", lacode);
          
          // High channel
          hfcode = ((data[1] & 0x1)<<7) | (data[0] >> 2);
          hacode = (data[1]>>1);
          
          // 80-1250hz
          //printf("HF : %x\n", hfcode);
          //printf("HA : %x\n", hacode);
          break;

      // Seems to be single wave mode
      case 5:
      case 6:
          //printf("Case 5-6\n");
          
          // Byte 0 is frequency and high/low select bit
          // check byte 0 bit 0
          // 1 indicates high channel
          high_f_select = data[0] & 1;

          if (high_f_select)
          {
              //printf("HF Bit ON\n");
              
              hfcode = (data[0]>>1);
              hacode = (data[1] & 0xF) << 3;
              
              //printf("LF is 160hz.");
              lfcode = 0;
              lacode = ( ((data[2] & 0x1)<<3) | ( (data[1]&0xE0)>>5 ) ) << 3;
              flo = 160.0f;
          }
              
          else
          {
              //printf("LF Bit ON\n");
              
              lfcode = (data[0]>>1);
              hacode = (data[1] & 0xF) << 3;
              
              //printf("HF is 320hz.");
              hfcode = 0;
              lacode = ( ((data[2] & 0x1)<<3) | ( (data[1]&0xE0)>>5 ) ) << 3;
              fhi = 320.0f;
          }
          
          if(data[1] & 0x10)
          {
              // Hi freqency amplitude disable
              hacode = 0;
          }
          
          if(data[2] & 0x2)
          {
              // Lo frequency amplitude disable
              lacode = 0;
          }
          
          // 80-1250hz
          //printf("HF : %x\n", hfcode);
          //printf("HA : %x\n", hacode);
          
          //printf("LF : %x\n", lfcode);
          //printf("LA : %x\n", lacode);
          break;

      // Some kind of operation codes? Also contains frequency
      case 7:
          //printf("Case 7\n");
          /*
          v18 = *data;
          v19 = v18 & 1;
          v20 = ((v18 >> 2) & 1) == 0;
          v21 = (*((unsigned short *)data + 1) >> 7) & 0x7F;

          if (v20)
              v22 = v21 | 0x80;
          else
              v22 = (v21 << 8) | 0x8000;

          if (v19)
              v23 = 24;
          else
              v23 = v22;
          amFmCodes[0] = v23;
          if (!v19)
              v22 = 24;
          amFmCodes[1] = v22;
          amFmCodes[2] = (data[2] >> 2) & 0x1F;
          amFmCodes[3] = (*(unsigned short *)(data + 1) >> 5) & 0x1F;
          amFmCodes[4] = data[1] & 0x1F;
          v12 = *data >> 3;*/
        return;
        break;

      default:
          break;
  }

  float out_amps = 0;

  if(!fhi)
  fhi = _get_high_frequency(hfcode);

  if(!flo)
  flo = _get_low_frequency(lfcode);

  ahi = _get_amplitude(hacode);
  alo = _get_amplitude(lacode);

  if(alo>=ahi)
  {
    alo = (alo>0) ? powf(alo, 0.65f) : 0;
    out_amps = 0xFFFF * alo;
    app_set_rumble(flo, (uint16_t) out_amps);
  }
  else
  {
    ahi = (ahi>0) ? powf(ahi, 0.65f) : 0;
    out_amps = 0xFFFF * ahi;
    app_set_rumble(fhi, (uint16_t) out_amps);
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
    //if (!app_compare_mac(global_loaded_settings.switch_host_mac, global_loaded_settings.paired_host_mac))
    //{
    //  app_save_host_mac();
    //}

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
  esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21, SWITCH_BT_REPORT_SIZE, _switch_input_buffer);
  vTaskDelay(1/portTICK_PERIOD_MS);
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
      uint8_t lr    : 1;
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
