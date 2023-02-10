/*!
 * @file main.cpp
 * @brief Main Function for the robot program
 *
 * The main function parses command line arguments and starts the appropriate
 * driver.
 */
#include <iostream>
#include <rt_spi.h>


unsigned char spi_mode = SPI_MODE_0;
unsigned char spi_bits_per_word = 8;
unsigned int spi_speed = 6000000;
uint8_t lsb = 0x01;

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
  std::cout << "Program started\n";
  std::cout << "\n";

  // Setup  and Open SPI
  int rv = 0;
  int spi_1_fd = open("/dev/spidev2.0", O_RDWR);
  if (spi_1_fd < 0) perror("[ERROR] Couldn't open spidev 2.0");

  rv = ioctl(spi_1_fd, SPI_IOC_WR_MODE, &spi_mode);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_wr_mode (1)");

  rv = ioctl(spi_1_fd, SPI_IOC_RD_MODE, &spi_mode);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_mode (1)");

  rv = ioctl(spi_1_fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits_per_word);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_wr_bits_per_word (1)");

  rv = ioctl(spi_1_fd, SPI_IOC_RD_BITS_PER_WORD, &spi_bits_per_word);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_bits_per_word (1)");

  rv = ioctl(spi_1_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_wr_max_speed_hz (1)");

  rv = ioctl(spi_1_fd, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_max_speed_hz (1)");

  rv = ioctl(spi_1_fd, SPI_IOC_RD_LSB_FIRST, &lsb);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_lsb_first (1)");


  uint8_t tx_buf[2];
  uint8_t rx_buf[2];

  uint8_t arr[2] = {1, 2};

  uint8_t *cmd_d = (uint8_t *)&arr;

  // copy into tx buffer flipping bytes
  for (int i = 0; i < 2; i++)
    tx_buf[i] = (cmd_d[i] >> 8) + ((cmd_d[i] & 0xff) << 8);

   // spi message struct
  struct spi_ioc_transfer spi_message[1];

  // zero message struct.
  memset(spi_message, 0, 1 * sizeof(struct spi_ioc_transfer));

  // set up message struct
  for (int i = 0; i < 1; i++) {
    spi_message[i].bits_per_word = 8;
    spi_message[i].cs_change = 1;
    spi_message[i].delay_usecs = 0;
    spi_message[i].len = 2;
    spi_message[i].rx_buf = (uint64_t)rx_buf;
    spi_message[i].tx_buf = (uint64_t)tx_buf;
  }

  // do spi communication
  rv = ioctl(spi_1_fd, SPI_IOC_MESSAGE(1),
                  &spi_message);
  (void)rv;

  std::cout << rx_buf[0] << "\n";
  std::cout << rx_buf[1] << "\n";

  return 0;
}
