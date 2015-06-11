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
#include "Arnl.h"
#include <assert.h>

int main(int argc, char **argv)
{
  Aria::init();
  Arnl::init();
  

  ArRobot robot;
  ArLMS2xx dummyLaser(1);
  ArMap arMap;
  ArSonarDevice sonar;
  robot.addRangeDevice(&sonar);

  // Create ARNL objects that use configuration parameters:
  ArLocalizationManager locManager(&robot, &arMap);
#ifdef ARNL
  ArLocalizationTask laserLocTask(&robot, &dummyLaser, &arMap);
  locManager.addLocalizationTask(&laserLocTask);
#endif
#ifdef SONARNL
  ArSonarLocalizationTask sonarLocTask(&robot, &sonar, &arMap);
  locManager.addLocalizationTask(&sonarLocTask);
#endif
#ifdef MOGS
  ArGPS gps;
  ArGPSLocalizationTask gpsLocTask(&robot, &gps, &arMap);
  locManager.addLocalizationTask(&gpsLocTask);
#endif

  ArPathPlanningTask pathTask(&robot, &dummyLaser, &sonar, &arMap);


  ArActionRatioInput teleop;
  teleop.addToConfig(Aria::getConfig(), "Teleop settings");
 
  // limit teleop maximum backward speed by default, increase rotation a bit
  // over ArActionRatioInput's defaults. Leave max forward speed at default
  // to be chosen by ArActionRatioInput.
  // fullThrottleForwar=0 fulThrottleBackwards=100, rotAtFullForwards=35,
  // rotAtFullBackwards=35, rotAtStopped=60
  teleop.setParameters(0, 290, 35, 35, 60);
  
  if(!Aria::getConfig()->writeFile("params/arnl.p")) Aria::exit(1);
  puts("wrote params/arnl.p");
  if(!Aria::getConfig()->writeFile("params/default-arnl.p")) Aria::exit(2);
  puts("wrote params/default-arnl.p");
  puts("ARNL makeParams finished.");
  Aria::exit(0);
  return 0;
}
