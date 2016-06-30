#ifndef ARTRANSFORM_H
#define ARTRANSFORM_H

#include "ariaTypedefs.h"
#include "ariaUtil.h"

/// Perform transforms between different coordinates
/** @ingroup UtilityClasses
*/
class ArTransform
{
public:
  /// Constructor
  ArTransform() 
    { 
      myX = 0;
      myY = 0; 
      myTh = 0;
      myCos = ArMath::cos(myTh);
      mySin = ArMath::sin(myTh);
    }
  /// Constructor, Sets the transform so points in this coord system
  /// transform to abs world coords

  ArTransform(ArPose pose) 
    { 
      setTransform(pose);
    }
  /// Constructor, sets the transform so that pose1 will be
  /// transformed to pose2
  ArTransform(ArPose pose1, ArPose pose2)
    {
      setTransform(pose1, pose2);
    }
  /// Destructor
  virtual ~ArTransform() {}

  /// Take the source pose and run the transform on it to put it into abs 
  /// coordinates
  /** 
      @param source the parameter to transform
      @return the source transformed into absolute coordinates
  */
  ArPose doTransform(ArPose source)
    {
      ArPose ret;
      ret.setX(myX + myCos * source.getX() + mySin * source.getY());
      ret.setY(myY + myCos * source.getY() - mySin * source.getX());
      ret.setTh(ArMath::addAngle(source.getTh(),myTh));      
      return ret;
    }
  /// Take the source pose and run the transform on it to put it into abs 
  /// coordinates
  /** 
      @param source the parameter to transform
      @return the source transformed into absolute coordinates
  */
  ArPoseWithTime doTransform(ArPoseWithTime source)
    {
      ArPoseWithTime ret;
      ret.setX(myX + myCos * source.getX() + mySin * source.getY());
      ret.setY(myY + myCos * source.getY() - mySin * source.getX());
      ret.setTh(ArMath::addAngle(source.getTh(),myTh));      
      ret.setTime(source.getTime());
      return ret;
    }

  /// Take the source pose and run the inverse transform on it, taking it from
  /// abs coords to local
  /** 
      The source and result can be the same
      @param source the parameter to transform
      @return the source transformed from absolute into local coords
  */
  ArPose doInvTransform(ArPose source)
    {
      ArPose ret;
      double tx = source.getX() - myX;
      double ty = source.getY() - myY;
      ret.setX(myCos * tx - mySin * ty);
      ret.setY(myCos * ty + mySin * tx);
      ret.setTh(ArMath::subAngle(source.getTh(),myTh));      
      return ret;
    }

  /// Take the source pose and run the inverse transform on it, taking it from
  /// abs coords to local
  /** 
      The source and result can be the same
      @param source the parameter to transform
      @return the source transformed from absolute into local coords
  */
  ArPoseWithTime doInvTransform(ArPoseWithTime source)
    {
      ArPoseWithTime ret;
      double tx = source.getX() - myX;
      double ty = source.getY() - myY;
      ret.setX(myCos * tx - mySin * ty);
      ret.setY(myCos * ty + mySin * tx);
      ret.setTh(ArMath::subAngle(source.getTh(),myTh));      
      ret.setTime(source.getTime());
      return ret;
    }


  /// Take a std::list of sensor readings and do the transform on it
  AREXPORT void doTransform(std::list<ArPose *> *poseList);
  /// Take a std::list of sensor readings and do the transform on it
  AREXPORT void doTransform(std::list<ArPoseWithTime *> *poseList);
  /// Sets the transform so points in this coord system transform to abs world coords
  AREXPORT void setTransform(ArPose pose);
  /// Sets the transform so that pose1 will be transformed to pose2
  AREXPORT void setTransform(ArPose pose1, ArPose pose2);
  /// Gets the transform x value (mm)
  double getX() { return myX; }
  /// Gets the transform y value (mm)
  double getY() { return myY; }
  /// Gets the transform angle value (degrees)
  double getTh() { return myTh; }
  /// Internal function for setting the transform from low level data not poses
  AREXPORT void setTransformLowLevel(double x, double y, double th);
protected:
  double myX;
  double myY;
  double myTh;
  double myCos;
  double mySin;
};


#endif // ARTRANSFORM_H
