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

public class ArServerModeDockTriangleBumpBackwards extends ArServerModeDock {
  private long swigCPtr;

  /* for internal use by swig only */
  public ArServerModeDockTriangleBumpBackwards(long cPtr, boolean cMemoryOwn) {
    super(ArnlJavaJNI.SWIGArServerModeDockTriangleBumpBackwardsUpcast(cPtr), cMemoryOwn);
    swigCPtr = cPtr;
  }

  /* for internal use by swig only */
  public static long getCPtr(ArServerModeDockTriangleBumpBackwards obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        ArnlJavaJNI.delete_ArServerModeDockTriangleBumpBackwards(swigCPtr);
      }
      swigCPtr = 0;
    }
    super.delete();
  }

  public void dock() {
    ArnlJavaJNI.ArServerModeDockTriangleBumpBackwards_dock(swigCPtr, this);
  }

  public void undock() {
    ArnlJavaJNI.ArServerModeDockTriangleBumpBackwards_undock(swigCPtr, this);
  }

  public void checkDock() {
    ArnlJavaJNI.ArServerModeDockTriangleBumpBackwards_checkDock(swigCPtr, this);
  }

  public void forceUnlock() {
    ArnlJavaJNI.ArServerModeDockTriangleBumpBackwards_forceUnlock(swigCPtr, this);
  }

  public void deactivate() {
    ArnlJavaJNI.ArServerModeDockTriangleBumpBackwards_deactivate(swigCPtr, this);
  }

  public boolean isDocked() {
    return ArnlJavaJNI.ArServerModeDockTriangleBumpBackwards_isDocked(swigCPtr, this);
  }

  public void enableDock() {
    ArnlJavaJNI.ArServerModeDockTriangleBumpBackwards_enableDock(swigCPtr, this);
  }

  public void disableDock() {
    ArnlJavaJNI.ArServerModeDockTriangleBumpBackwards_disableDock(swigCPtr, this);
  }

  public void beforeDriveInCallback() {
    ArnlJavaJNI.ArServerModeDockTriangleBumpBackwards_beforeDriveInCallback(swigCPtr, this);
  }

  public void afterDriveOutCallback() {
    ArnlJavaJNI.ArServerModeDockTriangleBumpBackwards_afterDriveOutCallback(swigCPtr, this);
  }

  public void pulloutCallback() {
    ArnlJavaJNI.ArServerModeDockTriangleBumpBackwards_pulloutCallback(swigCPtr, this);
  }

  public void setStallsAsBumps(boolean stallsAsBumps) {
    ArnlJavaJNI.ArServerModeDockTriangleBumpBackwards_setStallsAsBumps(swigCPtr, this, stallsAsBumps);
  }

  public boolean getStallsAsBumps() {
    return ArnlJavaJNI.ArServerModeDockTriangleBumpBackwards_getStallsAsBumps(swigCPtr, this);
  }

  public void addToConfig(ArConfig config) {
    ArnlJavaJNI.ArServerModeDockTriangleBumpBackwards_addToConfig(swigCPtr, this, ArConfig.getCPtr(config), config);
  }

}