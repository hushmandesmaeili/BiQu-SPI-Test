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
  std::cout << "\n";

  // SpiData _spiData;
  // SpiCommand _spiCommand;

  int spiCommand = 10;          // leg_controller
  int spi_command_drv = 4;      // rt_spi
  int* cmd = &spi_command_drv;

  std::cout << "&spiCommand init: " << &spiCommand << "\n";
  std::cout << "&spi_command_drv init: " << &spi_command_drv << "\n";
  std::cout << "&cmd init: " << &cmd << "\n";

  std::cout << "spiCommand init: " << spiCommand << "\n";
  std::cout << "spi_command_drv init: " << spi_command_drv << "\n";
  std::cout << "cmd init: " << cmd << "\n";
  
  memcpy(cmd, &spiCommand, sizeof(int));

  std::cout << "\n";

  std::cout << "&spiCommand final: " << &spiCommand << "\n";
  std::cout << "&spi_command_drv final: " << &spi_command_drv << "\n";
  std::cout << "&cmd final: " << &cmd << "\n";

  std::cout << "spiCommand final: " << spiCommand << "\n";
  std::cout << "spi_command_drv final: " << spi_command_drv << "\n";
  std::cout << "cmd final: " << cmd << "\n";

  // init_spi_biqu()

  return 0;
}
