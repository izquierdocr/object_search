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
#ifndef ARMULTIROBOTFLAGS_H
#define ARMULTIROBOTFLAGS_H

#include "ariaUtil.h"

/**
   This is used to say what bits in the multirobot capability flags
   are supported by a client.  This is so that as future flags are
   added the central software can know to not use the flags on older
   clients.
**/
class ArMultiRobotFlags1
{
public:
  enum Flags
  {
    WAITING_TO_FAIL = ArUtil::BIT0,
    END_OF_PATH = ArUtil::BIT1
  };
};

#endif // ARMULTIROBOTFLAGS_H
