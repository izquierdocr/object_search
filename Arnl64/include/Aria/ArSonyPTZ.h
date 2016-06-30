#ifndef ARSONYPTZ_H
#define ARSONYPTZ_H

#include "ariaTypedefs.h"
#include "ArBasePacket.h"
#include "ArPTZ.h"

/// A class for for making commands to send to the sony
/** There are only two functioning ways to put things into this packet, you
    MUST use thse, if you use anything else your commands won't work.  You 
    must use uByteToBuf and byte2ToBuf.  
*/
class ArSonyPacket: public ArBasePacket
{
public:
  /// Constructor
  AREXPORT ArSonyPacket(ArTypes::UByte2 bufferSize = 15);
  AREXPORT virtual ~ArSonyPacket();
  
  AREXPORT virtual void uByteToBuf(ArTypes::UByte val);
  AREXPORT virtual void byte2ToBuf(ArTypes::Byte2 val);
  /// This is a new function, read the details before you try to use it
  AREXPORT void byte2ToBufAtPos(ArTypes::Byte2 val, ArTypes::UByte2 pose);
};

class ArRobot;

/// A class to use the sony pan tilt zoom unit

class ArSonyPTZ : public ArPTZ
{
public:
  AREXPORT ArSonyPTZ(ArRobot *robot);
  AREXPORT virtual ~ArSonyPTZ();
  
  AREXPORT virtual bool init(void);
  AREXPORT virtual const char  *getTypeName() { return "sony"; }
protected:
  AREXPORT virtual bool pan_i(double degrees);
  AREXPORT virtual bool panRel_i(double degrees);
  AREXPORT virtual bool tilt_i(double degrees);
  AREXPORT virtual bool tiltRel_i(double degrees);
  AREXPORT virtual bool panTilt_i(double degreesPan, double degreesTilt);
  AREXPORT virtual bool panTiltRel_i(double degreesPan, double degreesTilt);
public:
  AREXPORT virtual bool canZoom(void) const { return true; }
  AREXPORT virtual bool zoom(int zoomValue);
  AREXPORT virtual bool zoomRel(int zoomValue);
protected:
  AREXPORT virtual double getPan_i(void) const { return myPan; }
  AREXPORT virtual double getTilt_i(void) const { return myTilt; }
public:
  AREXPORT virtual int getZoom(void) const { return myZoom; }


  AREXPORT virtual bool canGetFOV(void) { return true; }
  /// Gets the field of view at maximum zoom
  AREXPORT virtual double getFOVAtMaxZoom(void) { return 4.4; }
  /// Gets the field of view at minimum zoom
  AREXPORT virtual double getFOVAtMinZoom(void) { return 48.8; }

  AREXPORT bool backLightingOn(void);
  AREXPORT bool backLightingOff(void);
  //AREXPORT bool packetHandler(ArRobotPacket *packet);
  /* unused?
  enum {
    MAX_PAN = 95, ///< maximum degrees the unit can pan (either direction)
    MAX_TILT = 25, ///< maximum degrees the unit can tilt (either direction)
    MIN_ZOOM = 0, ///< minimum value for zoom
    MAX_ZOOM = 1023 ///< maximum value for zoom
  };
  */
  
  /// called automatically by Aria::init()
  ///@since 2.7.6
  ///@internal
#ifndef SWIG
  static void registerPTZType();
#endif

protected:
  void initializePackets(void);
  ArRobot *myRobot;
  double myPan;
  double myTilt;
  int myZoom;
  double myDegToTilt;
  double myDegToPan;
  ArSonyPacket myPacket;
  ArSonyPacket myZoomPacket; 
  ArSonyPacket myPanTiltPacket;

  ///@since 2.7.6
  static ArPTZ* create(size_t index, ArPTZParams params, ArArgumentParser *parser, ArRobot *robot);
  ///@since 2.7.6
  static ArPTZConnector::GlobalPTZCreateFunc ourCreateFunc;
};

#endif // ARSONYPTZ_H
