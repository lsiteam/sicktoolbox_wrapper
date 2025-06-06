///////////////////////////////////////////////////////////////////////////////
// a little program that dumps LMS scans to disk in ASCII.
//
// Copyright (C) 2010, Morgan Quigley
//
// License: BSD (see original block)
///////////////////////////////////////////////////////////////////////////////

#include <csignal>
extern "C" {
#include <stdint.h>  // compatibility
}
#include <cstdio>
#include <sicktoolbox/SickLMS2xx.hh>
#include <rclcpp/rclcpp.hpp>  // ROS 2 clock

using namespace SickToolbox;
using namespace std;

bool got_ctrlc = false;
void ctrlc_handler(int) {
  got_ctrlc = true;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);  // initialize ROS 2 context (optional here)
  rclcpp::Clock clock;

  if (argc != 4)
  {
    printf("Usage: log_scans DEVICE BAUD_RATE FILENAME\n");
    return 1;
  }

  FILE *log = fopen(argv[3], "w");
  if (!log)
  {
    fprintf(stderr, "couldn't open logfile %s\n", argv[3]);
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
  }

  try
  {
    while (!got_ctrlc)
    {
      sick_lms.GetSickScan(values, num_values);

      // print 12 values to console
      int inc = num_values / 11;
      printf("%5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d\n", 
             values[0],     values[inc], 
             values[2*inc], values[3*inc],
             values[4*inc], values[5*inc],
             values[6*inc], values[7*inc],
             values[8*inc], values[9*inc],
             values[10*inc], values[num_values-1]);

      // write timestamp and all values to disk
      fprintf(log, "%.6f ", clock.now().seconds());
      for (unsigned i = 0; i < num_values; i++)
        fprintf(log, "%d ", values[i]);
      fprintf(log, "\n");
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

  fclose(log);
  printf("success.\n");
  return 0;
}

