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
/* SWIG Wrapper for BaseArnl */

#ifdef SWIGPYTHON
%module(docstring="Python wrapper library for BaseArnl") BaseArnlPy
#else
# ifdef SWIGJAVA
%module(docstring="Java wrapper library for BaseArnl") BaseArnlJava
# else
%module(docstring="Wrapper library for BaseArnl") BaseArnl
# endif
#endif

#ifndef SWIGIMPORTED
#warning MAKING_BASEARNL
#define MAKING_BASEARNL 1
#else
#warning Imported wrapper_BaseArnl.i
#endif

%feature("autodoc", "1");

%rename (getPathPlanningLogLevel) ArPathPlanningTask::getLogLevel;

// include declarations
%{
#include "Aria.h"
#include "ArNetworking.h"
#include "Arnl.h"
#include "ArBaseLocalizationTask.h"
#include "ArPathPlanningInterface.h"
#include "ArPathPlanningTask.h"
#include "ArServerClasses.h"
#include "ArLocalizationManager.h"
#include "ArMultiRobot.h"
#include "ArMultiRobotFlags.h"
#include "ArMultiRobotPeer.h"
//#include "wrapper_ExtraClasses.h"
%}
%warnfilter(451) ArUtil;


%include "wrapper_common.i"



// include files to wrap
%include "arnlInternal.h"
%include "ArBaseLocalizationTask.h"
%include "ArPathPlanningInterface.h"
%include "ArPathPlanningTask.h"
%include "ArServerClasses.h"
%include "ArLocalizationManager.h"
%include "ArMultiRobot.h"
%include "ArMultiRobotFlags.h"
%include "ArMultiRobotPeer.h"

//#include "wrapper_ExtraClasses.h"

