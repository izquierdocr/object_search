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
/* SWIG Wrapper for Arnl */

#ifdef SWIGPYTHON
%module(docstring="Python wrapper library for ARNL") ArnlPy
#else
# ifdef SWIGJAVA
%module(docstring="Java wrapper library for ARNL") ArnlJava
# else
%module(docstring="Wrapper library for ARNL") Arnl
# endif
#endif

%feature("autodoc", "1");

// include declarations
%{
#define ARNL 1
#include "Aria.h"
#include "ArNetworking.h"
#include "Arnl.h"
#include "ArLocalizationTask.h"
#include "ArDockInterface.h"
#include "ArDocking.h"
#include "wrapper_ExtraClasses.h"
%}
%warnfilter(451) ArUtil;

#define ARNL 1

%include "wrapper_common.i"



/* Import BaseArnl wrapper */

#ifndef SWIG_IMPORTED_ARNL_BASE
%import "wrapper_BaseArnl.i"
#define SWIG_IMPORTED_ARNL_BASE 1
#endif


// include files to wrap
%include "ArLocalizationTask.h"
%include "ArDockInterface.h"
%include "ArDocking.h"

