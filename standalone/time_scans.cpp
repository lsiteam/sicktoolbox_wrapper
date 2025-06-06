///////////////////////////////////////////////////////////////////////////////
// this program is just a little test to make sure the laser is working.
// it's mostly just to familiarize myself with the sicktoolbox library.
// it's heavily lifted from the sicktoolbox lms_simple_app program.
//
// Copyright (C) 2008, Morgan Quigley
//
// License: BSD (see original block)
///////////////////////////////////////////////////////////////////////////////

#include <cstdlib>
#include <csignal>
#include <stdint.h>
#include <cstdio>
#include <sicktoolbox/SickLMS2xx.hh>
#include <rclcpp/rclcpp.hpp>

using namespace SickToolbox;
using namespace std;

bool got_ctrlc = false;
void ctrlc_handler(int)
{
  got_ctrlc = true;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Clock clock;

  if (argc != 3)
  {
    printf("Usage: print_scans DEVICE BAUD_RATE\n");
    return 1;
  }

  string lms_dev = argv[1];
  SickLMS2xx::sick_lms_2xx_baud_t desired_baud = SickLMS2xx::StringToSickBaud(argv[2]);
  if (desired_baud == SickLMS2xx::SICK_BAUD_UNKNOWN)
  {
    printf("bad baud rate. must be one of {9600, 19200, 38400, 500000}\n");
    return 1;
  }

  signal(SIGINT, ctrlc_handler);

  uint32_t values[SickLMS2xx::SICK_MAX_NUM_MEASUREMENTS] = {0};
  uint32_t num_values = 0;
  SickLMS2xx sick_lms(lms_dev);

  try
  {
    sick_lms.Initialize(desired_baud);
  }
  catch (...)
  {
    printf("initialize failed! are you using the correct device path?\n");
    return 1;
  }

  rclcpp::Time prev_scan_time = clock.now();
  try
  {
    while (!got_ctrlc)
    {
      sick_lms.GetSickScan(values, num_values);
      rclcpp::Time t = clock.now();
      double delta = (t - prev_scan_time).seconds();
      printf("%f (%f Hz)\n", delta, 1.0 / delta);
      prev_scan_time = t;
    }
  }
  catch (...)
  {
    printf("woah! error!\n");
  }

  try
  {
    sick_lms.Uninitialize();
  }
  catch (...)
  {
    printf("error during uninitialize\n");
    return 1;
  }

  printf("success.\n");
  return 0;
}

