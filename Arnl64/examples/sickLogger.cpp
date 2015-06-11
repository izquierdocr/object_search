/*

Copyright (c) 2014 Adept Technology Inc.
All rights reserved.  

Redistribution of this example source code, with or without modification, is 
permitted provided that the following conditions are met:   
-    Redistributions must retain the above copyright notice, 
     this list of conditions and the following disclaimer.  
-    Redistributions must be in source code form only

The information in this document is subject to change without notice and should
not be construed as a commitment by Adept Technology, Inc.

Adept Technology, Inc. makes no warranty as to the suitability of this material
for use by the recipient, and assumes no responsibility for any consequences
resulting from such use. 

Note: All other non-example software, including binary software objects
(libraries, programs), are prohibited from distribution under terms described
in LICENSE.txt (refer to LICENSE.txt for details).
*/
#include "Aria.h"
#include "Arnl.h"

/*
This program will let you joystick the robot around, and take logs for
the mapper while you drive, automatically... 

Control: 

Attach an analog joystick to the joyport on the robot, not on the
computer.  Now calibrate the joystick: Leaving the stick centered,
press the trigger button for a second or so, then release it. Now
rotate the stick around its extremes two or three times, holding it in
each corner for a second or two. You are now ready to drive.  You can
then drive the robot by holding the trigger and moving the joystick.
To make goals you can press the top button on the joystick (this
requires AROS1_5).

You could also attach USB joystick attached to robot computer (this
depends on having a robot equiped with an accessible usb port): To
drive the robot just press the trigger button and then move the
joystick how you wish the robot to move.  You can use the throttle on
the side of the joystick to control the maximum (and hence scaled)
speed at which the robot drives.  To make a goal you can press any of
the other buttons on the joystick itself (button 2, 3, or 4).

You could run this with the keyboard, but not very versatile.  Use the
arrow keys to control the robot, and press g to make a goal.
*/


int main(int argc, char **argv)
{
  Aria::init();
  Arnl::init();
  ArArgumentParser parser(&argc, argv);
  parser.loadDefaultArguments();

  ArRobot robot;
  ArRobotConnector robotConnector(&parser, &robot);
  ArLaserConnector laserConnector(&parser, &robot, &robotConnector);
  ArAnalogGyro analogGyro(&robot);

  // Always connect to the laser, and add half-degree increment and 180 degrees as default arguments for
  // laser
  parser.addDefaultArgument("-connectLaser -laserDegrees 180 -laserIncrement half");
  
  if(!robotConnector.connectRobot())
  {
    ArLog::log(ArLog::Terse, "sickLagger: Could not connect to the robot.");
    if(parser.checkHelpAndWarnUnparsed())
    {
      Aria::logOptions();
      Aria::exit(1);
    }
  }

  if(!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
  {
    Aria::logOptions();
    Aria::exit(1);
  }

  ArKeyHandler keyHandler;
  Aria::setKeyHandler(&keyHandler);
  robot.attachKeyHandler(&keyHandler);

#ifdef WIN32
  printf("Pausing 5 seconds so you can disconnect VNC if you are using it.\n");
  ArUtil::sleep(5000);
#endif


  std::string filename = "1scans.2d";
  if (argc > 1)
    filename = argv[1];
  printf("Logging to file %s\n", filename.c_str());
  
  
  

  ArActionGroupRatioDriveUnsafe group(&robot);
  group.activateExclusive();

  robot.runAsync(true);

  if(!laserConnector.connectLasers(false, false, true))
  {
    ArLog::log(ArLog::Terse, "sickLogger: Error: Could not connect to laser(s).  Use -help to list options.");
    Aria::exit(3);
  }


  // Allow some time for first set of laser reading to arrive
  ArUtil::sleep(500);
  

  // enable the motors, disable amigobot sounds
  robot.comInt(ArCommands::SONAR, 0);
  robot.comInt(ArCommands::ENABLE, 1);
  robot.comInt(ArCommands::SOUND, 32);
  robot.comInt(ArCommands::SOUNDTOG, 0);
  // enable the joystick driving from the one connected to the microcontroller
  robot.comInt(ArCommands::JOYDRIVE, 1);

  // Create the logging object
  // This must be created after the robot and laser are connected so it can get 
  // correct parameters from them.
  // The third argmument is how far the robot must travel before a new laser
  // scan is logged.
  // The fourth argument is how many degrees the robot must rotate before a new
  // laser scan is logged. The sixth boolean argument is whether to place a goal
  // when the g or G key is pressed (by adding a handler to the global
  // ArKeyHandler)  or when the robots joystick button is
  // pressed.
  ArLaser *laser = robot.findLaser(1);
  if(!laser)
  {
    ArLog::log(ArLog::Terse, "sickLogger: Error, not connected to any lasers.");
    Aria::exit(2);
  }
  ArLaserLogger logger(&robot, laser, 300, 25, filename.c_str(), true);

  // just hang out and wait for the end
  robot.waitForRunExit();

  Aria::exit(0);
}
