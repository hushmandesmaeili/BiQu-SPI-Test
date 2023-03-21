/*!
 * @file main.cpp
 * @brief Main Function for the robot program
 *
 * The main function parses command line arguments and starts the appropriate
 * driver.
 */
#include <iostream>
#include "../rt/include/rt_spi.h"
#include"../lcm-types/cpp/spi_command_t.hpp"
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
int main() {
  printf("Program started\n\n");

  // Setup  and Open SPI
  init_spi_biqu();

  // spi_command_t *command = new spi_command_t();
  spi_data_t data;
  spi_command_t command;
  // command.q_des_abad[0] = 1;
  // command.q_des_abad[1] = 2;
  // command.q_des_abad[2] = 3;
  // command.q_des_abad[3] = 4;

  data.q_abad[0] = 0;
  data.q_abad[1] = 0;
  data.q_abad[2] = 0;
  data.q_abad[3] = 0;

  spi_biqu_send_receive(&command, &data);

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
