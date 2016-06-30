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

%include <std_common.i>

%{
#include <list>
#include <stdexcept>
#include <cstddef>
%}


/* Wrap STL that ARNL uses: */
%include "std_vector.i"
#ifdef SWIGJAVA
%include "wrapper_std_list_java.i"
#else
%include "std_list.i"
#endif


/* Include functor classes for Python */
#ifdef SWIGPYTHON
%{
#include "wrapper_Functors.h"
%}
#endif
 


/* Import ARIA and ArNetworking wrappers: */

#ifndef SWIG_IMPORTED_ARIA
%import "../../Aria/include/wrapper.i"
#define SWIG_IMPORTED_ARIA 1
#endif

#ifndef SWIG_IMPORTED_ARNETWORKING
%import "../../Aria/ArNetworking/include/wrapper.i"
#define SWIG_IMPORTED_ARNETWORKING 1
#endif

  

/* In Java, we need to import the Aria and ArNetworking packages' namespaces */

#ifdef MAKING_BASEARNL

%typemap(javaimports) SWIGTYPE %{
  import com.mobilerobots.Aria.*;
  import com.mobilerobots.ArNetworking.*;
%}
%pragma(java) jniclassimports=%{
  import com.mobilerobots.Aria.*; 
  import com.mobilerobots.ArNetworking.*;
%}
%pragma(java) moduleimports=%{
  import com.mobilerobots.Aria.*; 
  import com.mobilerobots.ArNetworking.*;
%}

#else

%typemap(javaimports) SWIGTYPE %{
  import com.mobilerobots.Aria.*;
  import com.mobilerobots.ArNetworking.*;
  import com.mobilerobots.BaseArnl.*;
%}
%pragma(java) jniclassimports=%{
  import com.mobilerobots.Aria.*; 
  import com.mobilerobots.ArNetworking.*;
  import com.mobilerobots.BaseArnl.*;
%}
%pragma(java) moduleimports=%{
  import com.mobilerobots.Aria.*; 
  import com.mobilerobots.ArNetworking.*;
  import com.mobilerobots.BaseArnl.*;
%}

#endif

/* Instantiate STL container templates for specific use */
/* it's in aria... %template(ArPoseList) std::list<ArPose>;  */

