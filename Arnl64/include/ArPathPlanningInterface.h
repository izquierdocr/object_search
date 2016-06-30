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
#ifndef ARPATHPLANNINGINTERFACE_H
#define ARPATHPLANNINGINTERFACE_H

class ArPathPlanningInterface
{
public:
  ArPathPlanningInterface() {}
  virtual ~ArPathPlanningInterface() {}

  /// State of the path plan (accessible using getState()).
  enum PathPlanningState
  {
    NOT_INITIALIZED,    ///< Task not initialized 
    PLANNING_PATH,      ///< Planning the inital path
    MOVING_TO_GOAL,     ///< Moving to the goal.
    REACHED_GOAL,       ///< Reached the goal.
    FAILED_PLAN,        ///< Failed to plan a path to goal.
    FAILED_MOVE,        ///< Failed to reach goal after plan obtained.
    ABORTED_PATHPLAN,   ///< Aborted plan before done.
    INVALID             ///< Invalid state.
  };

  /// Set a new destination pose for the path planning task to plan to. 
  AREXPORT virtual bool   pathPlanToPose(ArPose goal, bool headingFlag, 
					 bool printFlag=true) = 0;
  /// Set a new goal (from the map) for the path planning task  to plan to.
  AREXPORT virtual bool   pathPlanToGoal(const char* goalname, 
					 bool strictGoalTypeMatching = true,
					 const char *goalType = "Goal") = 0;

  /// Cancel any current path following
  AREXPORT virtual void cancelPathPlan(void) = 0;

  /// Gets the state of path planning.
  virtual PathPlanningState getState(void) = 0;

  /// Gets a textual description of failure or current planning status.
  virtual void   getFailureString(char *str, size_t len) = 0;
  /// Gets a textual description of failure or current planning status.
  virtual void   getStatusString(char *str, size_t len) = 0;

  /// Gets the Aria map used in the path planning.
  virtual ArMapInterface* getAriaMap(void) = 0;

  /// Set the time the robot was not moving.
  virtual void   setLastMoveTime(double t) = 0;

  /// Adds a callback which will be called when goal reached.
  virtual void   addGoalDoneCB(ArFunctor1<ArPose>* functor)
  {
    myGoalDoneCBList.push_back(functor);
  }
  /// Removes a callback which used to be called when goal reached.
  virtual void   remGoalDoneCB(ArFunctor1<ArPose>* functor)
  {
    myGoalDoneCBList.remove(functor);
  }
  /// Adds a callback which will be called when goal failed.
  virtual void   addGoalFailedCB(ArFunctor1<ArPose>* functor)
  {
    myGoalFailedCBList.push_back(functor);
  }
  /// Removes a callback which used to be called when goal failed.
  virtual void   remGoalFailedCB(ArFunctor1<ArPose>* functor)
  {
    myGoalFailedCBList.remove(functor);
  }
  /// Adds a callback which will be called when goal is interrupted.
  virtual void   addGoalInterruptedCB(ArFunctor1<ArPose>* functor)
  {
    myGoalInterruptedCBList.push_back(functor);
  }
  /// Removes a callback which used to be called when goal is interrupted.
  virtual void   remGoalInterruptedCB(ArFunctor1<ArPose>* functor)
  {
    myGoalInterruptedCBList.remove(functor);
  }
  /// Adds a callback which will be called when there is a new goal
  virtual void   addNewGoalCB(ArFunctor1<ArPose>* functor)
  {
    myNewGoalCBList.push_back(functor);
  }
  /// Removes a callback which used to be called when there is a new goal
  virtual void   remNewGoalCB(ArFunctor1<ArPose>* functor)
  {
    myNewGoalCBList.remove(functor);
  }
  /// Adds a callback for when the goal is done, failed, or intterupted
  virtual void   addGoalFinishedCB(ArFunctor * functor)
  {
    myGoalFinishedCBList.push_back(functor);
  }
  /// Removes a callback for when the goal is done, failed, or intterupted
  virtual void   remGoalFinishedCB(ArFunctor * functor)
  {
    myGoalFinishedCBList.remove(functor);
  }

  /// Adds a plain callback for when there is a new goal
  virtual void addPlainNewGoalCB(ArFunctor *functor, int position = 50)
  { myPlainNewGoalCBList.addCallback(functor, position); }
  /// Removes a plain callback for when there is a new goal
  virtual void remPlainNewGoalCB(ArFunctor *functor)
  { myPlainNewGoalCBList.remCallback(functor); }
  /// Adds a plain callback for when a goal is finished, interrupted, or failed
  virtual void addPlainGoalFinishedCB(ArFunctor *functor, int position = 50)
  { myPlainGoalFinishedCBList.addCallback(functor, position); }
  /// Removes a plain callback for when a goal is finished, interrupted,
  /// or failed
  virtual void remPlainGoalFinishedCB(ArFunctor *functor)
  { myPlainGoalFinishedCBList.remCallback(functor); }
  /// Adds a plain callback for when goal is reached
  virtual void addPlainGoalDoneCB(ArFunctor *functor, int position = 50)
  { myPlainGoalDoneCBList.addCallback(functor, position); }
  /// Removes a plain callback for when the goal is reached
  virtual void remPlainGoalDoneCB(ArFunctor *functor)
  { myPlainGoalDoneCBList.remCallback(functor); }
  /// Adds a plain callback for when the goal fails
  virtual void addPlainGoalFailedCB(ArFunctor *functor, int position = 50)
  { myPlainGoalFailedCBList.addCallback(functor, position); }
  /// Removes a plain callback for when the goal fails
  virtual void remPlainGoalFailedCB(ArFunctor *functor)
  { myPlainGoalFailedCBList.remCallback(functor); }
  /// Adds a plain callback for when the goal is interrupted
  virtual void addPlainGoalInterruptedCB(ArFunctor *functor, 
					  int position = 50)
  { myPlainGoalInterruptedCBList.addCallback(functor, position); }
  /// Removes a plain callback for when the goal is interrupted
  virtual void remPlainGoalInterruptedCB(ArFunctor *functor)
  { myPlainGoalInterruptedCBList.remCallback(functor); }

  /// Add a callback to be notified when planning state changes
  virtual void    addStateChangeCB(ArFunctor* cb)
  {
    myStateChangeCallbacks.push_back(cb);
  }
  /// Remove a callback to be notified when planning state changes
  virtual void    remStateChangeCB(ArFunctor* cb)
  {
    myStateChangeCallbacks.remove(cb);
  }

  /// Add a callback function from the list to call when path blocked.
  virtual void    addBlockedPathCB(ArFunctor1<const std::list<ArPose>* >* cb)
  {
    myBlockedPathCallbacks.push_back(cb);
  }

  /// Remove a callback function from the list to call when path blocked.
  virtual void    remBlockedPathCB(ArFunctor1<const std::list<ArPose>* >* cb)
  {
    myBlockedPathCallbacks.remove(cb);
  }

  /// Adds a plain callback for when the path is blocked
  virtual void addPlainBlockedPathCB(ArFunctor *functor, int position = 50)
  { myPlainBlockedPathCBList.addCallback(functor, position); }
  /// Removes a plain callback for when the path is blocked
  virtual void remPlainBlockedPathCB(ArFunctor *functor)
  { myPlainBlockedPathCBList.remCallback(functor); }
  

  /** Invoke all the callbacks after a goal is done (completed successfully)
   *  @internal
   */
  AREXPORT virtual void invokeGoalDoneCB(ArPose goal);


  /** Invoke all the callbacks when there is a new goal
   *  @internal
   */
  AREXPORT virtual void invokeNewGoalCB(ArPose goal);

  /** Invoke all the callbacks after a goal is failed 
   *  @internal
   */
  AREXPORT virtual void invokeGoalFailedCB(ArPose goal);

  /** Invoke all the callbacks after a goal is interrupted
   *  @internal
   */
  AREXPORT virtual void invokeGoalInterruptedCB(ArPose goal);
  
  /** Invoke all the callbacks after a goal is interrupted 
   *  @internal
   */
  AREXPORT virtual void invokeStateChangedCB(void);

protected:
  std::list<ArFunctor1<ArPose>*>           myNewGoalCBList;
  std::list<ArFunctor*>                    myGoalFinishedCBList;
  std::list<ArFunctor1<ArPose>*>           myGoalDoneCBList;
  std::list<ArFunctor1<ArPose>*>           myGoalFailedCBList;
  std::list<ArFunctor1<ArPose>*>           myGoalInterruptedCBList;

  ArCallbackList myPlainNewGoalCBList;
  ArCallbackList myPlainGoalFinishedCBList;
  ArCallbackList myPlainBlockedPathCBList;
  ArCallbackList myPlainGoalDoneCBList;
  ArCallbackList myPlainGoalFailedCBList;
  ArCallbackList myPlainGoalInterruptedCBList;

  std::list<ArFunctor*>                    myStateChangeCallbacks;
  std::list<ArFunctor1<const std::list<ArPose>* >* > myBlockedPathCallbacks;


};

#endif 
