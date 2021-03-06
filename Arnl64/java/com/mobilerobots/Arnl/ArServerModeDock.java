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

public class ArServerModeDock extends ArServerMode {
  private long swigCPtr;

  /* for internal use by swig only */
  public ArServerModeDock(long cPtr, boolean cMemoryOwn) {
    super(ArnlJavaJNI.SWIGArServerModeDockUpcast(cPtr), cMemoryOwn);
    swigCPtr = cPtr;
  }

  /* for internal use by swig only */
  public static long getCPtr(ArServerModeDock obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        ArnlJavaJNI.delete_ArServerModeDock(swigCPtr);
      }
      swigCPtr = 0;
    }
    super.delete();
  }

  public static ArServerModeDock createDock(ArServerBase serverBase, ArRobot robot, ArLocalizationTask locTask, ArPathPlanningInterface pathTask, ArFunctor shutdownFunctor) {
    long cPtr = ArnlJavaJNI.ArServerModeDock_createDock__SWIG_0(ArServerBase.getCPtr(serverBase), serverBase, ArRobot.getCPtr(robot), robot, ArLocalizationTask.getCPtr(locTask), locTask, ArPathPlanningInterface.getCPtr(pathTask), pathTask, ArFunctor.getCPtr(shutdownFunctor), shutdownFunctor);
    return (cPtr == 0) ? null : new ArServerModeDock(cPtr, false);
  }

  public static ArServerModeDock createDock(ArServerBase serverBase, ArRobot robot, ArLocalizationTask locTask, ArPathPlanningInterface pathTask) {
    long cPtr = ArnlJavaJNI.ArServerModeDock_createDock__SWIG_1(ArServerBase.getCPtr(serverBase), serverBase, ArRobot.getCPtr(robot), robot, ArLocalizationTask.getCPtr(locTask), locTask, ArPathPlanningInterface.getCPtr(pathTask), pathTask);
    return (cPtr == 0) ? null : new ArServerModeDock(cPtr, false);
  }

  public ArDockInterface.State getState() {
    return ArDockInterface.State.swigToEnum(ArnlJavaJNI.ArServerModeDock_getState(swigCPtr, this));
  }

  public void dock() {
    ArnlJavaJNI.ArServerModeDock_dock(swigCPtr, this);
  }

  public void undock() {
    ArnlJavaJNI.ArServerModeDock_undock(swigCPtr, this);
  }

  public void checkDock() {
    ArnlJavaJNI.ArServerModeDock_checkDock(swigCPtr, this);
  }

  public void activateAsDocked() {
    ArnlJavaJNI.ArServerModeDock_activateAsDocked(swigCPtr, this);
  }

  public void checkDefault() {
    ArnlJavaJNI.ArServerModeDock_checkDefault(swigCPtr, this);
  }

  public void activate() {
    ArnlJavaJNI.ArServerModeDock_activate(swigCPtr, this);
  }

  public void deactivate() {
    ArnlJavaJNI.ArServerModeDock_deactivate(swigCPtr, this);
  }

  public void requestUnlock() {
    ArnlJavaJNI.ArServerModeDock_requestUnlock(swigCPtr, this);
  }

  public boolean getForcedDock() {
    return ArnlJavaJNI.ArServerModeDock_getForcedDock(swigCPtr, this);
  }

  public boolean isForcedDockAvailable() {
    return ArnlJavaJNI.ArServerModeDock_isForcedDockAvailable(swigCPtr, this);
  }

  public boolean isAutoDockAvailable() {
    return ArnlJavaJNI.ArServerModeDock_isAutoDockAvailable(swigCPtr, this);
  }

  public boolean hasGoToDockBeenSent() {
    return ArnlJavaJNI.ArServerModeDock_hasGoToDockBeenSent(swigCPtr, this);
  }

  public void gotoDock(boolean force) {
    ArnlJavaJNI.ArServerModeDock_gotoDock(swigCPtr, this, force);
  }

  public void requestForcedDock() {
    ArnlJavaJNI.ArServerModeDock_requestForcedDock(swigCPtr, this);
  }

  public void forceUnlock() {
    ArnlJavaJNI.ArServerModeDock_forceUnlock(swigCPtr, this);
  }

  public String getDockName() {
    return ArnlJavaJNI.ArServerModeDock_getDockName(swigCPtr, this);
  }

  public void setDockingVoltage(double dockingVoltage) {
    ArnlJavaJNI.ArServerModeDock_setDockingVoltage(swigCPtr, this, dockingVoltage);
  }

  public double getDockingVoltage() {
    return ArnlJavaJNI.ArServerModeDock_getDockingVoltage(swigCPtr, this);
  }

  public void setDoneChargingVoltage(double doneChargingVoltage) {
    ArnlJavaJNI.ArServerModeDock_setDoneChargingVoltage(swigCPtr, this, doneChargingVoltage);
  }

  public double getDoneChargingVoltage() {
    return ArnlJavaJNI.ArServerModeDock_getDoneChargingVoltage(swigCPtr, this);
  }

  public void setDoneChargingMinutes(int doneChargingMinutes) {
    ArnlJavaJNI.ArServerModeDock_setDoneChargingMinutes(swigCPtr, this, doneChargingMinutes);
  }

  public int getDoneChargingMinutes() {
    return ArnlJavaJNI.ArServerModeDock_getDoneChargingMinutes(swigCPtr, this);
  }

  public boolean getUseChargeState() {
    return ArnlJavaJNI.ArServerModeDock_getUseChargeState(swigCPtr, this);
  }

  public void setDoneChargingAtFloat(boolean doneChargingAtFloat) {
    ArnlJavaJNI.ArServerModeDock_setDoneChargingAtFloat(swigCPtr, this, doneChargingAtFloat);
  }

  public boolean getDoneChargingAtFloat() {
    return ArnlJavaJNI.ArServerModeDock_getDoneChargingAtFloat(swigCPtr, this);
  }

  public void setMinimumMinutesBetweenAutoDock(int minutesBetween) {
    ArnlJavaJNI.ArServerModeDock_setMinimumMinutesBetweenAutoDock(swigCPtr, this, minutesBetween);
  }

  public int getMinimumMinutesBetweenAutoDock() {
    return ArnlJavaJNI.ArServerModeDock_getMinimumMinutesBetweenAutoDock(swigCPtr, this);
  }

  public void setAutoDock(boolean autoDocking) {
    ArnlJavaJNI.ArServerModeDock_setAutoDock(swigCPtr, this, autoDocking);
  }

  public boolean getAutoDock() {
    return ArnlJavaJNI.ArServerModeDock_getAutoDock(swigCPtr, this);
  }

  public void addToConfig(ArConfig config) {
    ArnlJavaJNI.ArServerModeDock_addToConfig(swigCPtr, this, ArConfig.getCPtr(config), config);
  }

  public void addControlCommands(ArServerHandlerCommands handlerCommands) {
    ArnlJavaJNI.ArServerModeDock_addControlCommands(swigCPtr, this, ArServerHandlerCommands.getCPtr(handlerCommands), handlerCommands);
  }

  public void addInfoCommands(ArServerHandlerCommands handlerCommands, SWIGTYPE_p_ArServerHandlerPopup handlerPopup) {
    ArnlJavaJNI.ArServerModeDock_addInfoCommands(swigCPtr, this, ArServerHandlerCommands.getCPtr(handlerCommands), handlerCommands, SWIGTYPE_p_ArServerHandlerPopup.getCPtr(handlerPopup));
  }

  public void setGetOverrideMapNameFunctor(SWIGTYPE_p_ArRetFunctorT_char_const_p_t getOverrideMapNameFunctor) {
    ArnlJavaJNI.ArServerModeDock_setGetOverrideMapNameFunctor(swigCPtr, this, SWIGTYPE_p_ArRetFunctorT_char_const_p_t.getCPtr(getOverrideMapNameFunctor));
  }

  public void serverDock(ArServerClient client, ArNetPacket packet) {
    ArnlJavaJNI.ArServerModeDock_serverDock(swigCPtr, this, ArServerClient.getCPtr(client), client, ArNetPacket.getCPtr(packet), packet);
  }

  public void serverUndock(ArServerClient client, ArNetPacket packet) {
    ArnlJavaJNI.ArServerModeDock_serverUndock(swigCPtr, this, ArServerClient.getCPtr(client), client, ArNetPacket.getCPtr(packet), packet);
  }

  public void serverDockInfo(ArServerClient client, ArNetPacket packet) {
    ArnlJavaJNI.ArServerModeDock_serverDockInfo(swigCPtr, this, ArServerClient.getCPtr(client), client, ArNetPacket.getCPtr(packet), packet);
  }

  public void serverGetAutoDock(ArServerClient client, ArNetPacket packet) {
    ArnlJavaJNI.ArServerModeDock_serverGetAutoDock(swigCPtr, this, ArServerClient.getCPtr(client), client, ArNetPacket.getCPtr(packet), packet);
  }

  public void serverSetAutoDock(ArServerClient client, ArNetPacket packet) {
    ArnlJavaJNI.ArServerModeDock_serverSetAutoDock(swigCPtr, this, ArServerClient.getCPtr(client), client, ArNetPacket.getCPtr(packet), packet);
  }

  public void serverDockingAtToServer(ArServerClient client, ArNetPacket packet) {
    ArnlJavaJNI.ArServerModeDock_serverDockingAtToServer(swigCPtr, this, ArServerClient.getCPtr(client), client, ArNetPacket.getCPtr(packet), packet);
  }

  public void serverDockingAtFromServer(ArServerClient client, ArNetPacket packet) {
    ArnlJavaJNI.ArServerModeDock_serverDockingAtFromServer(swigCPtr, this, ArServerClient.getCPtr(client), client, ArNetPacket.getCPtr(packet), packet);
  }

  public void addStateChangedCB(ArFunctor functor, ArListPos.Pos position) {
    ArnlJavaJNI.ArServerModeDock_addStateChangedCB__SWIG_0(swigCPtr, this, ArFunctor.getCPtr(functor), functor, position.swigValue());
  }

  public void addStateChangedCB(ArFunctor functor) {
    ArnlJavaJNI.ArServerModeDock_addStateChangedCB__SWIG_1(swigCPtr, this, ArFunctor.getCPtr(functor), functor);
  }

  public void remStateChangedCB(ArFunctor functor) {
    ArnlJavaJNI.ArServerModeDock_remStateChangedCB(swigCPtr, this, ArFunctor.getCPtr(functor), functor);
  }

  public long getFailedGotoNum() {
    return ArnlJavaJNI.ArServerModeDock_getFailedGotoNum(swigCPtr, this);
  }

  public void addForcedDockCB(ArFunctor functor, int position) {
    ArnlJavaJNI.ArServerModeDock_addForcedDockCB__SWIG_0(swigCPtr, this, ArFunctor.getCPtr(functor), functor, position);
  }

  public void addForcedDockCB(ArFunctor functor) {
    ArnlJavaJNI.ArServerModeDock_addForcedDockCB__SWIG_1(swigCPtr, this, ArFunctor.getCPtr(functor), functor);
  }

  public void remForcedDockCB(ArFunctor functor) {
    ArnlJavaJNI.ArServerModeDock_remForcedDockCB(swigCPtr, this, ArFunctor.getCPtr(functor), functor);
  }

  public void addIdleDockCB(ArFunctor functor, int position) {
    ArnlJavaJNI.ArServerModeDock_addIdleDockCB__SWIG_0(swigCPtr, this, ArFunctor.getCPtr(functor), functor, position);
  }

  public void addIdleDockCB(ArFunctor functor) {
    ArnlJavaJNI.ArServerModeDock_addIdleDockCB__SWIG_1(swigCPtr, this, ArFunctor.getCPtr(functor), functor);
  }

  public void remIdleDockCB(ArFunctor functor) {
    ArnlJavaJNI.ArServerModeDock_remIdleDockCB(swigCPtr, this, ArFunctor.getCPtr(functor), functor);
  }

  public void addRequestedDockCB(ArFunctor functor, int position) {
    ArnlJavaJNI.ArServerModeDock_addRequestedDockCB__SWIG_0(swigCPtr, this, ArFunctor.getCPtr(functor), functor, position);
  }

  public void addRequestedDockCB(ArFunctor functor) {
    ArnlJavaJNI.ArServerModeDock_addRequestedDockCB__SWIG_1(swigCPtr, this, ArFunctor.getCPtr(functor), functor);
  }

  public void remRequestedDockCB(ArFunctor functor) {
    ArnlJavaJNI.ArServerModeDock_remRequestedDockCB(swigCPtr, this, ArFunctor.getCPtr(functor), functor);
  }

  public void addDrivingToDockCB(ArFunctor functor, int position) {
    ArnlJavaJNI.ArServerModeDock_addDrivingToDockCB__SWIG_0(swigCPtr, this, ArFunctor.getCPtr(functor), functor, position);
  }

  public void addDrivingToDockCB(ArFunctor functor) {
    ArnlJavaJNI.ArServerModeDock_addDrivingToDockCB__SWIG_1(swigCPtr, this, ArFunctor.getCPtr(functor), functor);
  }

  public void remDrivingToDockCB(ArFunctor functor) {
    ArnlJavaJNI.ArServerModeDock_remDrivingToDockCB(swigCPtr, this, ArFunctor.getCPtr(functor), functor);
  }

  public void addDrivingIntoDockCB(ArFunctor functor, int position) {
    ArnlJavaJNI.ArServerModeDock_addDrivingIntoDockCB__SWIG_0(swigCPtr, this, ArFunctor.getCPtr(functor), functor, position);
  }

  public void addDrivingIntoDockCB(ArFunctor functor) {
    ArnlJavaJNI.ArServerModeDock_addDrivingIntoDockCB__SWIG_1(swigCPtr, this, ArFunctor.getCPtr(functor), functor);
  }

  public void remDrivingIntoDockCB(ArFunctor functor) {
    ArnlJavaJNI.ArServerModeDock_remDrivingIntoDockCB(swigCPtr, this, ArFunctor.getCPtr(functor), functor);
  }

  public void addDockedCB(ArFunctor functor, int position) {
    ArnlJavaJNI.ArServerModeDock_addDockedCB__SWIG_0(swigCPtr, this, ArFunctor.getCPtr(functor), functor, position);
  }

  public void addDockedCB(ArFunctor functor) {
    ArnlJavaJNI.ArServerModeDock_addDockedCB__SWIG_1(swigCPtr, this, ArFunctor.getCPtr(functor), functor);
  }

  public void remDockedCB(ArFunctor functor) {
    ArnlJavaJNI.ArServerModeDock_remDockedCB(swigCPtr, this, ArFunctor.getCPtr(functor), functor);
  }

  public void addSingleShotDockedCB(ArFunctor functor, int position) {
    ArnlJavaJNI.ArServerModeDock_addSingleShotDockedCB__SWIG_0(swigCPtr, this, ArFunctor.getCPtr(functor), functor, position);
  }

  public void addSingleShotDockedCB(ArFunctor functor) {
    ArnlJavaJNI.ArServerModeDock_addSingleShotDockedCB__SWIG_1(swigCPtr, this, ArFunctor.getCPtr(functor), functor);
  }

  public void remSingleShotDockedCB(ArFunctor functor) {
    ArnlJavaJNI.ArServerModeDock_remSingleShotDockedCB(swigCPtr, this, ArFunctor.getCPtr(functor), functor);
  }

  public void addDockNowUnforcedCB(ArFunctor functor, int position) {
    ArnlJavaJNI.ArServerModeDock_addDockNowUnforcedCB__SWIG_0(swigCPtr, this, ArFunctor.getCPtr(functor), functor, position);
  }

  public void addDockNowUnforcedCB(ArFunctor functor) {
    ArnlJavaJNI.ArServerModeDock_addDockNowUnforcedCB__SWIG_1(swigCPtr, this, ArFunctor.getCPtr(functor), functor);
  }

  public void remDockNowUnforcedCB(ArFunctor functor) {
    ArnlJavaJNI.ArServerModeDock_remDockNowUnforcedCB(swigCPtr, this, ArFunctor.getCPtr(functor), functor);
  }

  public void addDockNowForcedCB(ArFunctor functor, int position) {
    ArnlJavaJNI.ArServerModeDock_addDockNowForcedCB__SWIG_0(swigCPtr, this, ArFunctor.getCPtr(functor), functor, position);
  }

  public void addDockNowForcedCB(ArFunctor functor) {
    ArnlJavaJNI.ArServerModeDock_addDockNowForcedCB__SWIG_1(swigCPtr, this, ArFunctor.getCPtr(functor), functor);
  }

  public void remDockNowForcedCB(ArFunctor functor) {
    ArnlJavaJNI.ArServerModeDock_remDockNowForcedCB(swigCPtr, this, ArFunctor.getCPtr(functor), functor);
  }

  public void addUndockingCB(ArFunctor functor, int position) {
    ArnlJavaJNI.ArServerModeDock_addUndockingCB__SWIG_0(swigCPtr, this, ArFunctor.getCPtr(functor), functor, position);
  }

  public void addUndockingCB(ArFunctor functor) {
    ArnlJavaJNI.ArServerModeDock_addUndockingCB__SWIG_1(swigCPtr, this, ArFunctor.getCPtr(functor), functor);
  }

  public void remUndockingCB(ArFunctor functor) {
    ArnlJavaJNI.ArServerModeDock_remUndockingCB(swigCPtr, this, ArFunctor.getCPtr(functor), functor);
  }

  public void addUndockedCB(ArFunctor functor, int position) {
    ArnlJavaJNI.ArServerModeDock_addUndockedCB__SWIG_0(swigCPtr, this, ArFunctor.getCPtr(functor), functor, position);
  }

  public void addUndockedCB(ArFunctor functor) {
    ArnlJavaJNI.ArServerModeDock_addUndockedCB__SWIG_1(swigCPtr, this, ArFunctor.getCPtr(functor), functor);
  }

  public void remUndockedCB(ArFunctor functor) {
    ArnlJavaJNI.ArServerModeDock_remUndockedCB(swigCPtr, this, ArFunctor.getCPtr(functor), functor);
  }

  public void setDockInfoLogLevel(ArLog.LogLevel logLevel) {
    ArnlJavaJNI.ArServerModeDock_setDockInfoLogLevel(swigCPtr, this, logLevel.swigValue());
  }

  public void setDockModeLogLevel(ArLog.LogLevel logLevel) {
    ArnlJavaJNI.ArServerModeDock_setDockModeLogLevel(swigCPtr, this, logLevel.swigValue());
  }

  public boolean subclassDoneCharging() {
    return ArnlJavaJNI.ArServerModeDock_subclassDoneCharging(swigCPtr, this);
  }

  public boolean subclassNotDoneCharging() {
    return ArnlJavaJNI.ArServerModeDock_subclassNotDoneCharging(swigCPtr, this);
  }

  public boolean subclassNeedsAutoDock(String reason, long lenOfReason) {
    return ArnlJavaJNI.ArServerModeDock_subclassNeedsAutoDock(swigCPtr, this, reason, lenOfReason);
  }

  public boolean subclassAddParamsDoneCharging(ArConfig config, String section) {
    return ArnlJavaJNI.ArServerModeDock_subclassAddParamsDoneCharging(swigCPtr, this, ArConfig.getCPtr(config), config, section);
  }

  public void subclassGetDockInfoString(String dockInfoStr, long dockInfoStrLength, boolean terse) {
    ArnlJavaJNI.ArServerModeDock_subclassGetDockInfoString(swigCPtr, this, dockInfoStr, dockInfoStrLength, terse);
  }

  public void baseAddParamsDoneCharging(ArConfig config, String section) {
    ArnlJavaJNI.ArServerModeDock_baseAddParamsDoneCharging(swigCPtr, this, ArConfig.getCPtr(config), config, section);
  }

  public void baseAddParamDockUntilDoneCharging(ArConfig config, String section) {
    ArnlJavaJNI.ArServerModeDock_baseAddParamDockUntilDoneCharging(swigCPtr, this, ArConfig.getCPtr(config), config, section);
  }

  public void baseAddParamMinutesToChargeFor(ArConfig config, String section) {
    ArnlJavaJNI.ArServerModeDock_baseAddParamMinutesToChargeFor(swigCPtr, this, ArConfig.getCPtr(config), config, section);
  }

  public void baseAddParamToChargeTo(ArConfig config, String section) {
    ArnlJavaJNI.ArServerModeDock_baseAddParamToChargeTo(swigCPtr, this, ArConfig.getCPtr(config), config, section);
  }

  public String getDockFileName() {
    return ArnlJavaJNI.ArServerModeDock_getDockFileName(swigCPtr, this);
  }

  public void setDockFileName(String fileName) {
    ArnlJavaJNI.ArServerModeDock_setDockFileName(swigCPtr, this, fileName);
  }

  public String getDockBaseDirectory() {
    return ArnlJavaJNI.ArServerModeDock_getDockBaseDirectory(swigCPtr, this);
  }

  public void setDockBaseDirectory(String baseDirectory) {
    ArnlJavaJNI.ArServerModeDock_setDockBaseDirectory(swigCPtr, this, baseDirectory);
  }

  public void restoreFromDockFile() {
    ArnlJavaJNI.ArServerModeDock_restoreFromDockFile(swigCPtr, this);
  }

  public void addPreWriteCallback(ArFunctor functor, int position) {
    ArnlJavaJNI.ArServerModeDock_addPreWriteCallback__SWIG_0(swigCPtr, this, ArFunctor.getCPtr(functor), functor, position);
  }

  public void addPreWriteCallback(ArFunctor functor) {
    ArnlJavaJNI.ArServerModeDock_addPreWriteCallback__SWIG_1(swigCPtr, this, ArFunctor.getCPtr(functor), functor);
  }

  public void remPreWriteCallback(ArFunctor functor) {
    ArnlJavaJNI.ArServerModeDock_remPreWriteCallback(swigCPtr, this, ArFunctor.getCPtr(functor), functor);
  }

  public void addPostWriteCallback(ArFunctor functor, int position) {
    ArnlJavaJNI.ArServerModeDock_addPostWriteCallback__SWIG_0(swigCPtr, this, ArFunctor.getCPtr(functor), functor, position);
  }

  public void addPostWriteCallback(ArFunctor functor) {
    ArnlJavaJNI.ArServerModeDock_addPostWriteCallback__SWIG_1(swigCPtr, this, ArFunctor.getCPtr(functor), functor);
  }

  public void remPostPostWriteCallback(ArFunctor functor) {
    ArnlJavaJNI.ArServerModeDock_remPostPostWriteCallback(swigCPtr, this, ArFunctor.getCPtr(functor), functor);
  }

  public void setDistanceCB(SWIGTYPE_p_ArRetFunctor2T_double_ArPose_ArPose_t functor) {
    ArnlJavaJNI.ArServerModeDock_setDistanceCB(swigCPtr, this, SWIGTYPE_p_ArRetFunctor2T_double_ArPose_ArPose_t.getCPtr(functor));
  }

}
