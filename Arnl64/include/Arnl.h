/*
Adept MobileRobots Advanced Robotics Navigation and Localization (ARNL)
Version 1.9.0

Copyright (C) 2004, 2005 ActivMedia Robotics LLC
Copyright (C) 2006-2009 MobileRobots Inc.
Copyright (C) 2010-2014 Adept Technology, Inc.

All Rights Reserved.

Adept MobileRobots does not make any representations about the
suitability of this software for any purpose.  It is provided "as is"
without express or implied warranty.

The license for this software is distributed as LICENSE.txt in the top
level directory.

robots@mobilerobots.com
Adept MobileRobots
10 Columbia Drive
Amherst, NH 03031
+1-603-881-7960

*/

#ifndef ARNL_H
#define ARNL_H

#include "Aria.h"
#include "ArNetworking.h"
#include "arnlInternal.h"
#include "ArPathPlanningTask.h"
#include "ArServerClasses.h"
#include "ArLocalizationManager.h"
#include "ArMultiRobot.h"
#include "ArMultiRobotPeer.h"
#include "ArCentralMultiRobot.h"

#ifdef ARNL
#include "ArLocalizationTask.h"
#include "ArDocking.h"
#include "ArCentralDockHelper.h"
#endif

#ifdef SONARNL
#include "ArSonarLocalizationTask.h"
#endif

#ifdef MOGS
#include "ArGPSLocalizationTask.h"
#endif

#endif

