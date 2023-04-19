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

#ifdef linux
  #include <sys/timerfd.h>
#endif
#include <unistd.h>
#include <cmath>
#include "Timer.h"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

float _period = 0.002;
volatile bool _running = false;
float _lastRuntime = 0;
float _lastPeriodTime = 0;
// float _maxPeriod = 0;
// float _maxRuntime = 0;

int _counter = 0;
int _maxCounter = 2000;     // 4 seconds duration at _period = 0.002

// Declare 2D vector to store data from CSV file
std::vector<std::vector<float>> data;

// spi_data_t data;
// spi_command_t command;

std::vector<spi_command_t> commands;

void runSpi() {
  std::cout << "Send: " << _counter << "\n";
}

void import_csv() {
  std::ifstream infile("data.csv");
  std::string line;

  while (std::getline(infile, line)) {
    // Create a stringstream from the line and use getline() again to 
    // split the line into individual values separated by commas
    std::stringstream ss(line);
    std::string value_str;
    std::vector<float> row;
    while (std::getline(ss, value_str, ',')) {
      float value = std::stof(value_str);
      row.push_back(value);
    }

    // Add the row to the 2D vector
    data.push_back(row);
  }

  // // Test code for import
  // std::cout << "Data length (rows): " << data.size() << "\n";
  // std::cout << data[0][1] << "\n";
  // std::cout << data[1][1] << "\n";
  // std::cout << data[2][1] << "\n";
}

void convert_csvdata_to_spicommand() {

  for (int i = 0; i < data.size(); i++) {
    spi_command_t command;
    command.q_des_abad[0] = data[i][1];
    command.q_des_abad[1] = 0;
    command.q_des_abad[2] = 0;
    command.q_des_abad[3] = 0;

    command.q_des_hip[0] = data[i][2];
    command.q_des_hip[1] = 0;
    command.q_des_hip[2] = 0;
    command.q_des_hip[3] = 0;

    command.q_des_knee[0] = data[i][3];
    command.q_des_knee[1] = 0;
    command.q_des_knee[2] = 0;
    command.q_des_knee[3] = 0;

    command.qd_des_abad[0] = data[i][4];
    command.qd_des_abad[1] = 0;
    command.qd_des_abad[2] = 0;
    command.qd_des_abad[3] = 0;

    command.qd_des_hip[0] = data[i][5];
    command.qd_des_hip[1] = 0;
    command.qd_des_hip[2] = 0;
    command.qd_des_hip[3] = 0;

    command.qd_des_knee[0] = data[i][6];
    command.qd_des_knee[1] = 0;
    command.qd_des_knee[2] = 0;
    command.qd_des_knee[3] = 0;

    command.kp_abad[0] = data[i][7];
    command.kp_abad[1] = 0;
    command.kp_abad[2] = 0;
    command.kp_abad[3] = 0;

    command.kp_hip[0] = data[i][8];
    command.kp_hip[1] = 0;
    command.kp_hip[2] = 0;
    command.kp_hip[3] = 0;

    command.kp_knee[0] = data[i][10];
    command.kp_knee[1] = 0;
    command.kp_knee[2] = 0;
    command.kp_knee[3] = 0;

    command.kd_abad[0] = data[i][11];
    command.kd_abad[1] = 0;
    command.kd_abad[2] = 0;
    command.kd_abad[3] = 0;

    command.kd_hip[0] = data[i][12];
    command.kd_hip[1] = 0;
    command.kd_hip[2] = 0;
    command.kd_hip[3] = 0;

    command.kd_knee[0] = data[i][13];
    command.kd_knee[1] = 0;
    command.kd_knee[2] = 0;
    command.kd_knee[3] = 0;

    command.tau_abad_ff[0] = data[i][14];
    command.tau_abad_ff[1] = 0;
    command.tau_abad_ff[2] = 0;
    command.tau_abad_ff[3] = 0;

    command.tau_hip_ff[0] = data[i][15];
    command.tau_hip_ff[1] = 0;
    command.tau_hip_ff[2] = 0;
    command.tau_hip_ff[3] = 0;

    command.tau_knee_ff[0] = data[i][16];
    command.tau_knee_ff[1] = 0;
    command.tau_knee_ff[2] = 0;
    command.tau_knee_ff[3] = 0;

    command.flags[0] = data[i][17];
    command.flags[1] = 0;
    command.flags[2] = 0;
    command.flags[3] = 0;

    commands.push_back(command);
  }
}

/*!
 *
 */
int main()
{
  printf("Program started\n\n");

  // // Setup  and Open SPI
  // init_spi_biqu();

  // // spi_command_t *command = new spi_command_t();
  // spi_data_t data;
  // spi_command_t command;
  // command.q_des_abad[0] = 1.4;
  // command.q_des_abad[1] = 2.4;
  // command.q_des_abad[2] = 3.6;
  // command.q_des_abad[3] = 4.2;

  // command.q_des_hip[0] = 5.4;
  // command.q_des_hip[1] = 6.2;
  // command.q_des_hip[2] = 7.8;
  // command.q_des_hip[3] = 8.1;

  // command.q_des_knee[0] = 9.0;
  // command.q_des_knee[1] = 10.5;
  // command.q_des_knee[2] = 11.8;
  // command.q_des_knee[3] = 12.9;

  // command.qd_des_abad[0] = 13.1;
  // command.qd_des_abad[1] = 14.2;
  // command.qd_des_abad[2] = 15.3;
  // command.qd_des_abad[3] = 16.4;

  // command.qd_des_hip[0] = 17.5;
  // command.qd_des_hip[1] = 18.6;
  // command.qd_des_hip[2] = 19.7;
  // command.qd_des_hip[3] = 20.8;

  // command.qd_des_knee[0] = 21.9;
  // command.qd_des_knee[1] = 22.0;
  // command.qd_des_knee[2] = 23.1;
  // command.qd_des_knee[3] = 24.2;

  // command.kp_abad[0] = 25.3;
  // command.kp_abad[1] = 26.4;
  // command.kp_abad[2] = 27.5;
  // command.kp_abad[3] = 28.6;

  // command.kp_hip[0] = 29.7;
  // command.kp_hip[1] = 30.8;
  // command.kp_hip[2] = 31.9;
  // command.kp_hip[3] = 32.0;

  // command.kp_knee[0] = 33.1;
  // command.kp_knee[1] = 34.2;
  // command.kp_knee[2] = 35.3;
  // command.kp_knee[3] = 36.4;

  // command.kd_abad[0] = 37.5;
  // command.kd_abad[1] = 38.6;
  // command.kd_abad[2] = 39.7;
  // command.kd_abad[3] = 40.8;

  // command.kd_hip[0] = 41.9;
  // command.kd_hip[1] = 42.0;
  // command.kd_hip[2] = 43.1;
  // command.kd_hip[3] = 44.2;

  // command.kd_knee[0] = 45.3;
  // command.kd_knee[1] = 46.4;
  // command.kd_knee[2] = 47.5;
  // command.kd_knee[3] = 48.6;

  // command.tau_abad_ff[0] = 49.7;
  // command.tau_abad_ff[1] = 50.8;
  // command.tau_abad_ff[2] = 51.9;
  // command.tau_abad_ff[3] = 52.0;

  // command.tau_hip_ff[0] = 53.1;
  // command.tau_hip_ff[1] = 54.2;
  // command.tau_hip_ff[2] = 55.3;
  // command.tau_hip_ff[3] = 56.4;

  // command.tau_knee_ff[0] = 57.5;
  // command.tau_knee_ff[1] = 58.6;
  // command.tau_knee_ff[2] = 59.7;
  // command.tau_knee_ff[3] = 60.8;

  // command.flags[0] = 61;
  // command.flags[1] = 62;
  // command.flags[2] = 63;
  // command.flags[3] = 64;

  // data.q_abad[0] = 0;
  // data.q_abad[1] = 0;
  // data.q_abad[2] = 0;
  // data.q_abad[3] = 0;

  // spi_biqu_send_receive(&command, &data);
  // spi_biqu_send_receive(&command, &data);
  // spi_biqu_send_receive(&command, &data);


  //******TEST FOR PERIODIC SEND******//
//   _running = true;
//   _counter = 0;

// #ifdef linux
//   auto timerFd = timerfd_create(CLOCK_MONOTONIC, 0);
// #endif
//   int seconds = (int)_period;
//   int nanoseconds = (int)(1e9 * std::fmod(_period, 1.f));

//   Timer t;

// #ifdef linux
//   itimerspec timerSpec;
//   timerSpec.it_interval.tv_sec = seconds;
//   timerSpec.it_value.tv_sec = seconds;
//   timerSpec.it_value.tv_nsec = nanoseconds;
//   timerSpec.it_interval.tv_nsec = nanoseconds;

//   timerfd_settime(timerFd, 0, &timerSpec, nullptr);
// #endif
//   unsigned long long missed = 0;

//   printf("[PeriodicTask] Start %s (%d s, %d ns)\n", "spi", seconds,
//          nanoseconds);
//   while (_running) {
//     _lastPeriodTime = (float)t.getSeconds();
//     t.start();
//     runSpi();
//     _lastRuntime = (float)t.getSeconds();
// #ifdef linux
//     int m = read(timerFd, &missed, sizeof(missed));
//     (void)m;
// #endif
//     // _maxPeriod = std::max(_maxPeriod, _lastPeriodTime);
//     // _maxRuntime = std::max(_maxRuntime, _lastRuntime);
//     if (++_counter > _maxCounter) _running = false;
//   }


  //******TEST FOR import_csv******//
  import_csv();
  convert_csvdata_to_spicommand();

  // Testing spi_command conversion
  std::cout << commands[0].q_des_abad[0] << "\n";
  std::cout << commands[1].q_des_abad[0] << "\n";
  std::cout << commands[2].q_des_abad[0] << "\n";

  return 0;
}


