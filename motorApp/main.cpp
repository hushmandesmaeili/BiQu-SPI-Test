/*!
 * @file main.cpp
 * @brief Main Function for the robot program
 *
 * The main function parses command line arguments and starts the appropriate
 * driver.
 */
#include <iostream>
#include <rt_spi.h>

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

  SpiData _spiData;
  SpiCommand _spiCommand;

  // init_spi_biqu()

  return 0;
}
