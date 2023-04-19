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
int _maxCounter = 12130;     // 4 seconds duration at _period = 0.002

// Declare 2D vector to store data from CSV file
std::vector<std::vector<float>> dataCsv;

spi_data_t data;
// spi_command_t command;

std::vector<spi_command_t> commands;

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
    dataCsv.push_back(row);
  }

  // Test code for import
  // std::cout << "Data length (rows): " << dataCsv.size() << "\n";
  // std::cout << dataCsv[0][1] << "\n";
  // std::cout << dataCsv[1][1] << "\n";
  // std::cout << dataCsv[2][1] << "\n";
}

void convert_csvdata_to_spicommand() {

  for (int i = 0; i < dataCsv.size(); i++) {
    spi_command_t command;
    command.q_des_abad[0] = dataCsv[i][1];
    command.q_des_abad[1] = 0;
    command.q_des_abad[2] = 0;
    command.q_des_abad[3] = 0;

    command.q_des_hip[0] = dataCsv[i][2];
    command.q_des_hip[1] = 0;
    command.q_des_hip[2] = 0;
    command.q_des_hip[3] = 0;

    command.q_des_knee[0] = dataCsv[i][3];
    command.q_des_knee[1] = 0;
    command.q_des_knee[2] = 0;
    command.q_des_knee[3] = 0;

    command.qd_des_abad[0] = dataCsv[i][4];
    command.qd_des_abad[1] = 0;
    command.qd_des_abad[2] = 0;
    command.qd_des_abad[3] = 0;

    command.qd_des_hip[0] = dataCsv[i][5];
    command.qd_des_hip[1] = 0;
    command.qd_des_hip[2] = 0;
    command.qd_des_hip[3] = 0;

    command.qd_des_knee[0] = dataCsv[i][6];
    command.qd_des_knee[1] = 0;
    command.qd_des_knee[2] = 0;
    command.qd_des_knee[3] = 0;

    command.kp_abad[0] = dataCsv[i][7];
    command.kp_abad[1] = 0;
    command.kp_abad[2] = 0;
    command.kp_abad[3] = 0;

    command.kp_hip[0] = dataCsv[i][8];
    command.kp_hip[1] = 0;
    command.kp_hip[2] = 0;
    command.kp_hip[3] = 0;

    command.kp_knee[0] = dataCsv[i][9];
    command.kp_knee[1] = 0;
    command.kp_knee[2] = 0;
    command.kp_knee[3] = 0;

    command.kd_abad[0] = dataCsv[i][10];
    command.kd_abad[1] = 0;
    command.kd_abad[2] = 0;
    command.kd_abad[3] = 0;

    command.kd_hip[0] = dataCsv[i][11];
    command.kd_hip[1] = 0;
    command.kd_hip[2] = 0;
    command.kd_hip[3] = 0;

    command.kd_knee[0] = dataCsv[i][12];
    command.kd_knee[1] = 0;
    command.kd_knee[2] = 0;
    command.kd_knee[3] = 0;

    command.tau_abad_ff[0] = dataCsv[i][13];
    command.tau_abad_ff[1] = 0;
    command.tau_abad_ff[2] = 0;
    command.tau_abad_ff[3] = 0;

    command.tau_hip_ff[0] = dataCsv[i][14];
    command.tau_hip_ff[1] = 0;
    command.tau_hip_ff[2] = 0;
    command.tau_hip_ff[3] = 0;

    command.tau_knee_ff[0] = dataCsv[i][15];
    command.tau_knee_ff[1] = 0;
    command.tau_knee_ff[2] = 0;
    command.tau_knee_ff[3] = 0;

    command.flags[0] = dataCsv[i][16];
    command.flags[1] = 0;
    command.flags[2] = 0;
    command.flags[3] = 0;

    commands.push_back(command);
  }
  // std::cout << "Completed dataCsv to spi_command conversion" << "\n";
}

void debug_commands_vect(int command_num) {
   // Testing spi_command conversion
  std::cout << commands[command_num].q_des_abad[0] << ", "
            << commands[command_num].q_des_hip[0] << ", "
            << commands[command_num].q_des_knee[0] << ", "
            << commands[command_num].qd_des_abad[0] << ", "
            << commands[command_num].qd_des_hip[0] << ", "
            << commands[command_num].qd_des_knee[0] << ", "
            << commands[command_num].kp_abad[0] << ", "
            << commands[command_num].kp_hip[0] << ", "
            << commands[command_num].kp_knee[0] << ", "
            << commands[command_num].kd_abad[0] << ", "
            << commands[command_num].kd_hip[0] << ", "
            << commands[command_num].kd_knee[0] << ", "
            << commands[command_num].tau_abad_ff[0] << ", "
            << commands[command_num].tau_hip_ff[0] << ", "
            << commands[command_num].tau_knee_ff[0] << ", "
            << commands[command_num].flags[0]
            << "\n";
}

void runSpi() {
  std::cout << "Send: " << _counter << "\n";
  spi_biqu_send_receive(&(commands[_counter]), &data);

  // std::cout << "Send: " << _counter << ", ";
  // debug_commands_vect(_counter);
}

/*!
 *
 */
int main()
{
  printf("Program started\n\n");

  // Setup  and Open SPI
  printf("Initiating SPI\n\n");
  init_spi_biqu();

  printf("Importing CSV data\n\n");
  //******TEST FOR import_csv******//
  import_csv();
  convert_csvdata_to_spicommand();


  //******TEST FOR PERIODIC SEND******//
  _running = true;
  _counter = 0;

#ifdef linux
  auto timerFd = timerfd_create(CLOCK_MONOTONIC, 0);
#endif
  int seconds = (int)_period;
  int nanoseconds = (int)(1e9 * std::fmod(_period, 1.f));

  Timer t;

#ifdef linux
  itimerspec timerSpec;
  timerSpec.it_interval.tv_sec = seconds;
  timerSpec.it_value.tv_sec = seconds;
  timerSpec.it_value.tv_nsec = nanoseconds;
  timerSpec.it_interval.tv_nsec = nanoseconds;

  timerfd_settime(timerFd, 0, &timerSpec, nullptr);
#endif
  unsigned long long missed = 0;

  printf("[PeriodicTask] Start %s (%d s, %d ns)\n", "spi", seconds,
         nanoseconds);
  while (_running) {
    _lastPeriodTime = (float)t.getSeconds();
    t.start();
    runSpi();
    _lastRuntime = (float)t.getSeconds();
#ifdef linux
    int m = read(timerFd, &missed, sizeof(missed));
    (void)m;
#endif
    // _maxPeriod = std::max(_maxPeriod, _lastPeriodTime);
    // _maxRuntime = std::max(_maxRuntime, _lastRuntime);
    if (++_counter > _maxCounter) _running = false;
  }


 
  return 0;
}


