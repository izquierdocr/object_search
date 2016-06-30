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
/* ***************************************************************************
 * 
 * File: ArRobotPoseProb.h
 * 
 * Function: Header file for the robotposeprob.cpp file.
 *
 * Created:  George V. Paul. gvp@activmedia.com. December 6 2002.
 *
 *****************************************************************************/
#ifndef ARROBOTPOSESAMPLES_H
#define ARROBOTPOSESAMPLES_H

#include <stdio.h>
#include <math.h>
#include "Aria.h"
#include "ArOccGrid.h"
#include "ArSystemError.h"
#include "ArRobotPoseProb.h"
/* 
  @class ArRobotPoseSamples.
  @internal
  @brief Class holds the array of pose+probabilties forming a sample.
*/
class ArRobotPoseSamples
{

public:
  
  /// Base constructor.
  AREXPORT ArRobotPoseSamples(void);
  /// Constructor with the number of samples.
  AREXPORT ArRobotPoseSamples(int n);
  /// Constructor with number of samples and initial pose.
  AREXPORT ArRobotPoseSamples(int noofsamples, ArPose pose);
  /// Base destructor.
  AREXPORT ~ArRobotPoseSamples(void);

  /// Set the samples to lie in a pose boundary with uniform probability.
  AREXPORT bool setUniformSamples(double x, double y, double t,
			 double rx, double ry, double rt);
  /// Set the samples to lie in a gaussian dist around a given pose.
  AREXPORT bool setGaussianSamples(double rx, double ry, double rt,
			  double stdx, double stdy, double stdt);
  /// Peturb samples about existing locations.
  AREXPORT bool peturbSamples(double rx, double ry, double rt);
  /// Set the Maximum probability slot with these values.
  void setMaxPoseProb(double a, double b, double c, double d)
  {
    myMaxPoseProb.setPose(a,b,c);
    myMaxPoseProb.setProb(d);
  }
  /// Set the Maximum probability slot with these values.
  void setMaxPoseProb(ArPose p, double d)
  {
    myMaxPoseProb.setPose(p); 
    myMaxPoseProb.setProb(d);
  }
  /// Set the Maximum probability slot with these values.
  void setMaxPoseProb(ArPose p) 
  {
    myMaxPoseProb.setPose(p.getX(), p.getY(), p.getTh());
  }
  /// Set the goodness from the maximum probable pose from the MCL.
  void setMaxScore(double a){myMaxScore = a;}
  /// Get the most probable pose from the MCL calculation.
  ArRobotPoseProb getMaxPoseProb(void) {return myMaxPoseProb;}
  /// Predict the poses in the samples after the given move.
  AREXPORT bool predictPoseFromMove(double delr, double delt, double dela,
			   double kr, double kt, double kd);
  /// Resample the probability distribution after sensor matching.
  AREXPORT bool resampleToUniform(void);
  /// Find the pose with maximum probability value in the samples.
  AREXPORT bool computeMaxPoseProb(ArRobotPoseProb& max);
  /// Find the mean pose of the samples.
  AREXPORT bool computeMeanPoseProb(ArRobotPoseProb& mean);
  /// Adds a random value to the poses of the sample.
  AREXPORT void randomizeSamples(double stdx, double stdy, double stdt);
  /// Get the current number of samples.
  int  getNumSamples(void){return myNumSamples;}
  /// Get the current maximum goodness score.
  double getMaxScore(void){return myMaxScore;}
  /// Prints the samples.
  AREXPORT void fprintSamples(int skip, char* name);
  /// Prints the samples.
  AREXPORT void printSamples(int skip);
  /// Returns the i-th sample.
  ArRobotPoseProb* getRPSample(int i){return &mySamples[i];}
  /// Correct all samples by the given pose.
  AREXPORT void correctSamples(double x, double y, double t);
  /// Make a histogram to show on the GUI.
  AREXPORT bool makeHistogramAndCumSum(double*& hist, double*& cumsum);
  /// Special for changing numsamples.
  AREXPORT bool resampleToUniform(ArRobotPoseProb* oldSamples, int oldNumSamples);
  /// Store the statistics.
  void setMeanAndVariance(double xMean, double yMean, double tMean,
			  double xVar, double yVar, double tVar)
  { 
    myMeanPose = ArPose(xMean, yMean, tMean);
    myVariance = ArPose(xVar, yVar, tVar);
  }
  /// Get the stats.
  void getMeanAndVariance(ArPose& mean, ArPose& variance)
  { 
    mean = myMeanPose;
    variance = myVariance;
  }
  /// Reseed part of the samples with new pose.
  AREXPORT bool reseedSamples(ArPose center, double factor,
		     double stdx, double stdy, double stdt, double score);
  /// Get a set of lines defining the samples.
  AREXPORT std::vector<ArPose> getSamplePoses(int m);
  /// Gets the bounding box around samples.
  AREXPORT std::vector<ArPose> getBoundingBox(void);

private:

  int              myNumSamples;
  double           myMaxScore;
  ArRobotPoseProb  myMaxPoseProb;
  ArRobotPoseProb* mySamples;
  // The mean and variance of the samples after localization.
  ArPose           myMeanPose;
  ArPose           myVariance;
};

#endif // ARROBOTPOSESAMPLES_H
