/*!
 * @file main.cpp
 * @brief Main Function for the robot program
 *
 * The main function parses command line arguments and starts the appropriate
 * driver.
 */
#include <iostream>
#include "../rt/include/rt_spi.h"
#include "../lcm-types/cpp/spi_command_t.hpp"
#include "../lcm-types/cpp/spi_data_t.hpp"

// unsigned char spi_mode = SPI_MODE_0;
// unsigned char spi_bits_per_word = 8;
// unsigned int spi_speed = 6000000;
// uint8_t lsb = 0x01;

// uint32_t reverseBits(uint32_t b)
// {
//   b = (b & 0xFFFF0000) >> 16 | (b & 0x0000FFFF) << 16;
//   b = (b & 0xFF00FF00) >> 8  | (b & 0x00FF00FF) << 8;
//   b = (b & 0xF0F0F0F0) >> 4  | (b & 0x0F0F0F0F) << 4;
//   b = (b & 0xCCCCCCCC) >> 2  | (b & 0x33333333) << 2;
//   b = (b & 0xAAAAAAAA) >> 1  | (b & 0x55555555) << 1;
//   return b;
// }

// uint32_t reverseBytes(uint32_t b)
// {
//   b = ((b & 0xFF000000) >> 24) | ((b & 0x00FF0000) >> 8) | ((b & 0x0000FF00) << 8) | ((b & 0x000000FF) << 24);
//   return b;
// }

// runSpi() {
//   spi_command_t* cmd = get_spi_command();
//   spi_data_t* data = get_spi_data();

//   memcpy(cmd, &_spiCommand, sizeof(spi_command_t));
//   spi_biqu_driver_run();
//   memcpy(&_spiData, data, sizeof(spi_data_t));

//   _spiLcm.publish("spi_data", data);
//   _spiLcm.publish("spi_command", cmd);
// }

/*!
 *
 */
int main()
{
  printf("Program started\n\n");

  // Setup  and Open SPI
  init_spi_biqu();

  // spi_command_t *command = new spi_command_t();
  spi_data_t data;
  spi_command_t command;
  command.q_des_abad[0] = 1.4;
  command.q_des_abad[1] = 2.4;
  command.q_des_abad[2] = 3.6;
  command.q_des_abad[3] = 4.2;

  command.q_des_hip[0] = 5.4;
  command.q_des_hip[1] = 6.2;
  command.q_des_hip[2] = 7.8;
  command.q_des_hip[3] = 8.1;

  command.q_des_knee[0] = 9.0;
  command.q_des_knee[1] = 10.5;
  command.q_des_knee[2] = 11.8;
  command.q_des_knee[3] = 12.9;

  command.qd_des_abad[0] = 13.1;
  command.qd_des_abad[1] = 14.2;
  command.qd_des_abad[2] = 15.3;
  command.qd_des_abad[3] = 16.4;

  command.qd_des_hip[0] = 17.5;
  command.qd_des_hip[1] = 18.6;
  command.qd_des_hip[2] = 19.7;
  command.qd_des_hip[3] = 20.8;

  command.qd_des_knee[0] = 21.9;
  command.qd_des_knee[1] = 22.0;
  command.qd_des_knee[2] = 23.1;
  command.qd_des_knee[3] = 24.2;

  command.kp_abad[0] = 25.3;
  command.kp_abad[1] = 26.4;
  command.kp_abad[2] = 27.5;
  command.kp_abad[3] = 28.6;

  command.kp_hip[0] = 29.7;
  command.kp_hip[1] = 30.8;
  command.kp_hip[2] = 31.9;
  command.kp_hip[3] = 32.0;

  command.kp_knee[0] = 33.1;
  command.kp_knee[1] = 34.2;
  command.kp_knee[2] = 35.3;
  command.kp_knee[3] = 36.4;

  command.kd_abad[0] = 37.5;
  command.kd_abad[1] = 38.6;
  command.kd_abad[2] = 39.7;
  command.kd_abad[3] = 40.8;

  command.kd_hip[0] = 41.9;
  command.kd_hip[1] = 42.0;
  command.kd_hip[2] = 43.1;
  command.kd_hip[3] = 44.2;

  command.kd_knee[0] = 45.3;
  command.kd_knee[1] = 46.4;
  command.kd_knee[2] = 47.5;
  command.kd_knee[3] = 48.6;

  command.tau_abad_ff[0] = 49.7;
  command.tau_abad_ff[1] = 50.8;
  command.tau_abad_ff[2] = 51.9;
  command.tau_abad_ff[3] = 52.0;

  command.tau_hip_ff[0] = 53.1;
  command.tau_hip_ff[1] = 54.2;
  command.tau_hip_ff[2] = 55.3;
  command.tau_hip_ff[3] = 56.4;

  command.tau_knee_ff[0] = 57.5;
  command.tau_knee_ff[1] = 58.6;
  command.tau_knee_ff[2] = 59.7;
  command.tau_knee_ff[3] = 60.8;

  command.flags[0] = 61;
  command.flags[1] = 62;
  command.flags[2] = 63;
  command.flags[3] = 64;

  data.q_abad[0] = 0;
  data.q_abad[1] = 0;
  data.q_abad[2] = 0;
  data.q_abad[3] = 0;

  spi_biqu_send_receive(&command, &data);
  // spi_biqu_send_receive(&command, &data);


  // uint16_t tx_buf[4];
  // uint16_t rx_buf[6];

  // uint16_t arr[4] = {1, 2, 3, 4};
  // uint16_t receivearr[6] = {0, 0, 0, 0, 0, 0};

  // uint16_t *cmd_d = (uint16_t *)&arr;
  // uint16_t *data_d = (uint16_t *)&receivearr;
  // std::cout << cmd_d << "\n";

  // // copy into tx buffer flipping bits
  // for (int i = 0; i < 4; i++)
  //   tx_buf[i] = reverseBits(cmd_d[i]);//(cmd_d[i] >> 8) + ((cmd_d[i]) << 8);

  // std::cout << "Command from Rpi" << "\n";
  // std::cout << arr[0] << "\n";
  // std::cout << arr[1] << "\n";
  // std::cout << arr[2] << "\n";
  // std::cout << arr[3] << "\n";

  // // std::cout << tx_buf[0] << "\n";
  // // std::cout << tx_buf[1] << "\n";
  // // std::cout << tx_buf[2] << "\n";
  // // std::cout << tx_buf[3] << "\n";

  //  // spi message struct
  // struct spi_ioc_transfer spi_message[1];

  // // zero message struct.
  // memset(spi_message, 0, 1 * sizeof(struct spi_ioc_transfer));

  // // set up message struct
  // for (int i = 0; i < 1; i++) {
  //   spi_message[i].bits_per_word = spi_bits_per_word;
  //   spi_message[i].cs_change = 1;
  //   spi_message[i].delay_usecs = 0;
  //   spi_message[i].len = 8;
  //   spi_message[i].rx_buf = (__uint128_t)rx_buf;
  //   spi_message[i].tx_buf = (__uint128_t)tx_buf;
  // }

  // // do spi communication
  // rv = ioctl(spi_1_fd, SPI_IOC_MESSAGE(1),
  //                 &spi_message);
  // if (rv == 1) perror("[ERROR] cannot send message");
  // std::cout << "rv = " << rv << "\n";
  // (void)rv;

  // for (int i = 0; i < 4; i++)  // BiQu = 58, from spine_biqu_data_t entries * 2 bytes/entry
  //   data_d[i] = reverseBytes(reverseBits(rx_buf[i]));//(rx_buf[i] >> 8) + ((rx_buf[i]) << 8);
  // int temp1 = data_d[0];
  // int temp2 = data_d[1];
  // data_d[0] = data_d[2];
  // data_d[1] = data_d[3];
  // data_d[2] = temp1;
  // data_d[3] = temp2;

  // // std::cout << rx_buf[0] << "\n";
  // // std::cout << rx_buf[1] << "\n";
  // std::cout << "Array from Teensy: data_d" << "\n";
  // std::cout << receivearr[0] << "\n";
  // std::cout << receivearr[1] << "\n";
  // std::cout << receivearr[2] << "\n";
  // std::cout << receivearr[3] << "\n";

  // // std::cout << "rx_buf" << "\n";
  // // std::cout << rx_buf[0] << "\n";
  // // std::cout << rx_buf[1] << "\n";
  // // std::cout << rx_buf[2] << "\n";
  // // std::cout << rx_buf[3] << "\n";
  // // std::cout << rx_buf[4] << "\n";

  return 0;
}
