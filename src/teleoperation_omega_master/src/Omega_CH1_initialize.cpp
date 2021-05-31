
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>

#include <stdio.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>

#include "dhdc.h"
#include "drdc.h"

#define REFRESH_INTERVAL  0.1   // sec

#define SERIAL_NUMBER_OMEGA3 11415 // Omega3
#define SERIAL_NUMBER_OMEGA6 11359 // Omega6

class Master_CH1_Init
{
public:
	ros::NodeHandlePtr node_;
	ros::NodeHandlePtr pnode_;

	int init()
  {
    // Use global namespace for node
    node_ = ros::NodeHandlePtr(new ros::NodeHandle(""));

    // Use private namespace for parameters
    pnode_ = ros::NodeHandlePtr(new ros::NodeHandle("~"));

    double px, py, pz;
    double fx, fy, fz;
    double freq   = 0.0;
    double t1,t0  = dhdGetTime ();
    int    done   = 0;

    // message
    int major, minor, release, revision;
    dhdGetSDKVersion (&major, &minor, &release, &revision);
    printf ("Force Dimension - Automatic Initialization %d.%d.%d.%d\n", major, minor, release, revision);
    printf ("(C) 2014 Force Dimension\n");
    printf ("All Rights Reserved.\n\n");

    // required to change asynchronous operation mode
    dhdEnableExpertMode ();

    // open the first available device
    if (drdOpenID (0) < 0) {
      printf ("error: cannot open device (%s)\n", dhdErrorGetLastStr ());
      dhdSleep (2.0);
      return -1;
    }
    printf("device ID : %d\n",drdGetDeviceID());

    // print out device identifier
    if (!drdIsSupported()) {
      printf ("unsupported device\n");
      printf ("exiting...\n");
      dhdSleep (2.0);
      drdClose ();
      return -1;
    }
    printf ("%s haptic device detected\n\n", dhdGetSystemName());

    // perform auto-initialization
    printf ("initializing...\r");
    fflush (stdout);
    if (drdAutoInit () < 0) {
      drdStop ();
      printf ("error: auto-initialization failed (%s)\n", dhdErrorGetLastStr ());
      dhdSleep (2.0);
      return -1;
    }

    // perform initialization check (optional)
    printf ("checking initialization...\r");
    fflush (stdout);
    if (drdCheckInit () < 0) {
      drdStop ();
      printf ("error: device initialization check failed (%s)\n", dhdErrorGetLastStr ());
      dhdSleep (2.0);
      return -1;
    }

    // report success
    printf ("device successfully initialized\n\n");

    // stop regulation (and leave force enabled)
    drdStop (true);

    // display instructions
    printf ("press BUTTON or 'q' to quit\n\n");

    // loop while the button is not pushed

    while (!done) {

      // apply zero force
      if (dhdSetForceAndTorqueAndGripperForce (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0) < DHD_NO_ERROR) {
        printf ("error: cannot set force (%s)\n", dhdErrorGetLastStr());
        done = 1;
      }

      // display refresh rate and position at 10Hz
      t1 = dhdGetTime ();
      if ((t1-t0) > REFRESH_INTERVAL) {

        // retrieve information to display
        freq = dhdGetComFreq ();
        t0   = t1;

        // write down position
        if (dhdGetPosition (&px, &py, &pz) < 0) {
          printf ("error: cannot read position (%s)\n", dhdErrorGetLastStr());
          done = 1;
        }
        if (dhdGetForce (&fx, &fy, &fz) < 0) {
          printf ("error: cannot read force (%s)\n", dhdErrorGetLastStr());
          done = 1;
        }
        printf ("p (%+0.03f %+0.03f %+0.03f) m  |  f (%+0.01f %+0.01f %+0.01f) N  |  freq [%0.02f kHz]       \r", px, py, pz, fx, fy, fz, freq);

        // test for exit condition
        if (dhdGetButtonMask()) done = 1;
        if (dhdKbHit()) {
          switch (dhdKbGet()) {
          case 'q': done = 1; break;
          }
        }
      }
    }

    // close the connection
    printf ("cleaning up...                                                           \n");
    drdClose ();

    // happily exit
    printf ("\ndone.\n");


    return 0;
	}


};
int main(int argc, char **argv)
{
        ros::init(argc, argv, "Omega_CH1_initialize");
        Master_CH1_Init _Master_CH1_Init;
        if(_Master_CH1_Init.init())
	{
          ROS_FATAL("Master CH1 initialization failed");
          return -1;
	}
	return 0;
}
