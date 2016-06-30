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
/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 1.3.40
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package com.mobilerobots.Arnl;

  import com.mobilerobots.Aria.*;
  import com.mobilerobots.ArNetworking.*;
  import com.mobilerobots.BaseArnl.*;

public class ArServerModeDockSimulator extends ArServerModeDockTriangleBump {
  private long swigCPtr;

  /* for internal use by swig only */
  public ArServerModeDockSimulator(long cPtr, boolean cMemoryOwn) {
    super(ArnlJavaJNI.SWIGArServerModeDockSimulatorUpcast(cPtr), cMemoryOwn);
    swigCPtr = cPtr;
  }

  /* for internal use by swig only */
  public static long getCPtr(ArServerModeDockSimulator obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        ArnlJavaJNI.delete_ArServerModeDockSimulator(swigCPtr);
      }
      swigCPtr = 0;
    }
    super.delete();
  }

  public boolean isDocked() {
    return ArnlJavaJNI.ArServerModeDockSimulator_isDocked(swigCPtr, this);
  }

  public void enableDock() {
    ArnlJavaJNI.ArServerModeDockSimulator_enableDock(swigCPtr, this);
  }

  public void disableDock() {
    ArnlJavaJNI.ArServerModeDockSimulator_disableDock(swigCPtr, this);
  }

  public void checkDock() {
    ArnlJavaJNI.ArServerModeDockSimulator_checkDock(swigCPtr, this);
  }

}
