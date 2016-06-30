#ifndef ARROBOTTYPES_H
#define ARROBOTTYPES_H


#include "ariaUtil.h"
#include "ArRobotParams.h"

/** @cond INCLUDE_INTERNAL_ROBOT_PARAM_CLASSES */

class ArRobotGeneric : public ArRobotParams
{
public:
  AREXPORT ArRobotGeneric(const char *dir="");
  AREXPORT virtual ~ArRobotGeneric() {}
};

class ArRobotAmigo : public ArRobotParams
{
public:

  AREXPORT ArRobotAmigo(const char *dir="");
  AREXPORT virtual ~ArRobotAmigo() {}
};

class ArRobotAmigoSh : public ArRobotParams
{
public:

  AREXPORT ArRobotAmigoSh(const char *dir="");
  AREXPORT virtual ~ArRobotAmigoSh() {}
};

class ArRobotP2AT : public ArRobotParams
{
public:
  AREXPORT ArRobotP2AT(const char *dir="");
  AREXPORT virtual ~ArRobotP2AT() {}
};

class ArRobotP2AT8 : public ArRobotParams
{
public:
  AREXPORT ArRobotP2AT8(const char *dir="");
  AREXPORT virtual ~ArRobotP2AT8() {}
};

class ArRobotP2AT8Plus : public ArRobotParams
{
public:
  AREXPORT ArRobotP2AT8Plus(const char *dir="");
  AREXPORT virtual ~ArRobotP2AT8Plus() {}
};

class ArRobotP2IT : public ArRobotParams
{
public:
  AREXPORT ArRobotP2IT(const char *dir="");
  AREXPORT virtual ~ArRobotP2IT() {}
};

class ArRobotP2DX : public ArRobotParams
{
public:
  AREXPORT ArRobotP2DX(const char *dir="");
  AREXPORT virtual ~ArRobotP2DX() {}
};

class ArRobotP2DXe : public ArRobotParams
{
public:
  AREXPORT ArRobotP2DXe(const char *dir="");
  AREXPORT virtual ~ArRobotP2DXe() {}
};

class ArRobotP2DF : public ArRobotParams
{
public:
  AREXPORT ArRobotP2DF(const char *dir="");
  AREXPORT virtual ~ArRobotP2DF() {}
};

class ArRobotP2D8 : public ArRobotParams
{
public:
  AREXPORT ArRobotP2D8(const char *dir="");
  AREXPORT virtual ~ArRobotP2D8() {}
};

class ArRobotP2D8Plus : public ArRobotParams
{
public:
  AREXPORT ArRobotP2D8Plus(const char *dir="");
  AREXPORT virtual ~ArRobotP2D8Plus() {}
};

class ArRobotP2CE : public ArRobotParams
{
public:
  AREXPORT ArRobotP2CE(const char *dir="");
  AREXPORT virtual ~ArRobotP2CE() {}
};

class ArRobotP2PP : public ArRobotParams
{
public:
  AREXPORT ArRobotP2PP(const char *dir="");
  AREXPORT virtual ~ArRobotP2PP() {}
};

class ArRobotP2PB : public ArRobotParams
{
public:
  AREXPORT ArRobotP2PB(const char *dir="");
  AREXPORT virtual ~ArRobotP2PB() {}
};


class ArRobotP3AT : public ArRobotParams
{
public:
  AREXPORT ArRobotP3AT(const char *dir="");
  AREXPORT virtual ~ArRobotP3AT() {}
};


class ArRobotP3DX : public ArRobotParams
{
public:
  AREXPORT ArRobotP3DX(const char *dir="");
  AREXPORT virtual ~ArRobotP3DX() {}
};

class ArRobotPerfPB : public ArRobotParams
{
public:
  AREXPORT ArRobotPerfPB(const char *dir="");
  AREXPORT virtual ~ArRobotPerfPB() {}
};

class ArRobotPerfPBPlus : public ArRobotParams
{
public:
  AREXPORT ArRobotPerfPBPlus(const char *dir="");
  AREXPORT virtual ~ArRobotPerfPBPlus() {}
};

class ArRobotPion1M : public ArRobotParams
{
public:
  AREXPORT ArRobotPion1M(const char *dir="");
  AREXPORT virtual ~ArRobotPion1M() {}
};

class ArRobotPion1X : public ArRobotParams
{
public:
  AREXPORT ArRobotPion1X(const char *dir="");
  AREXPORT virtual ~ArRobotPion1X() {}
};

class ArRobotPsos1M : public ArRobotParams
{
public:
  AREXPORT ArRobotPsos1M(const char *dir="");
  AREXPORT virtual ~ArRobotPsos1M() {}
};

class ArRobotPsos43M : public ArRobotParams
{
public:
  AREXPORT ArRobotPsos43M(const char *dir="");
  AREXPORT virtual ~ArRobotPsos43M() {}
};

class ArRobotPsos1X : public ArRobotParams
{
public:
  AREXPORT ArRobotPsos1X(const char *dir="");
  AREXPORT virtual ~ArRobotPsos1X() {}
};

class ArRobotPionAT : public ArRobotParams
{
public:
  AREXPORT ArRobotPionAT(const char *dir="");
  AREXPORT virtual ~ArRobotPionAT() {}
};

class ArRobotMapper : public ArRobotParams
{
public:
  AREXPORT ArRobotMapper(const char *dir="");
  AREXPORT virtual ~ArRobotMapper() {}
};

class ArRobotPowerBot : public ArRobotParams
{
public:
  AREXPORT ArRobotPowerBot(const char *dir="");
  AREXPORT virtual ~ArRobotPowerBot() {}
};

class ArRobotP3DXSH : public ArRobotParams
{
 public:
  AREXPORT ArRobotP3DXSH(const char *dir="");
  AREXPORT virtual ~ArRobotP3DXSH() {}
};

class ArRobotP3ATSH : public ArRobotParams
{
 public:
  AREXPORT ArRobotP3ATSH(const char *dir="");
  AREXPORT virtual ~ArRobotP3ATSH() {}
};

class ArRobotP3ATIWSH : public ArRobotParams
{
 public:
  AREXPORT ArRobotP3ATIWSH(const char *dir="");
  AREXPORT virtual ~ArRobotP3ATIWSH() {}
};

class ArRobotPatrolBotSH : public ArRobotParams
{
 public:
  AREXPORT ArRobotPatrolBotSH(const char *dir="");
  AREXPORT virtual ~ArRobotPatrolBotSH() {}
};

class ArRobotPeopleBotSH : public ArRobotParams
{
 public:
  AREXPORT ArRobotPeopleBotSH(const char *dir="");
  AREXPORT virtual ~ArRobotPeopleBotSH() {}
};

class ArRobotPowerBotSH : public ArRobotParams
{
 public:
  AREXPORT ArRobotPowerBotSH(const char *dir="");
  AREXPORT virtual ~ArRobotPowerBotSH() {}
};

class ArRobotWheelchairSH : public ArRobotParams
{
 public:
  AREXPORT ArRobotWheelchairSH(const char *dir="");
  AREXPORT virtual ~ArRobotWheelchairSH() {}
};

class ArRobotPowerBotSHuARCS : public ArRobotParams
{
 public:
  AREXPORT ArRobotPowerBotSHuARCS(const char *dir="");
  AREXPORT virtual ~ArRobotPowerBotSHuARCS() {}
};

class ArRobotSeekur : public ArRobotParams
{
 public:
  AREXPORT ArRobotSeekur(const char *dir="");
  AREXPORT virtual ~ArRobotSeekur() {}
};

/// @since Aria 2.7.2
class ArRobotMT400 : public ArRobotParams
{
 public:
  AREXPORT ArRobotMT400(const char *dir="");
  AREXPORT virtual ~ArRobotMT400() {}
};

/// @since Aria 2.7.2
class ArRobotResearchPB : public ArRobotParams
{
 public:
  AREXPORT ArRobotResearchPB(const char *dir="");
  AREXPORT virtual ~ArRobotResearchPB() {}
};

/// @since Aria 2.7.2
class ArRobotSeekurJr : public ArRobotParams
{
 public:
  AREXPORT ArRobotSeekurJr(const char *dir="");
  AREXPORT virtual ~ArRobotSeekurJr() {}
};

/// @since Aria 2.7.4
class ArRobotP3DXSH_lms1xx : public ArRobotP3DXSH
{
public: 
  AREXPORT ArRobotP3DXSH_lms1xx(const char *dir="");
};

/// @since Aria 2.7.4
class ArRobotP3ATSH_lms1xx : public ArRobotP3ATSH
{
public: 
  AREXPORT ArRobotP3ATSH_lms1xx(const char *dir="");
};

/// @since Aria 2.7.4
class ArRobotPeopleBotSH_lms1xx : public ArRobotPeopleBotSH
{
public: 
  AREXPORT ArRobotPeopleBotSH_lms1xx(const char *dir="");
};

/// @since Aria 2.7.4
class ArRobotP3DXSH_lms500 : public ArRobotP3DXSH
{
public: 
  AREXPORT ArRobotP3DXSH_lms500(const char *dir="");
};

/// @since Aria 2.7.4
class ArRobotP3ATSH_lms500 : public ArRobotP3ATSH
{
public: 
  AREXPORT ArRobotP3ATSH_lms500(const char *dir="");
};

/// @since Aria 2.7.4
class ArRobotPeopleBotSH_lms500 : public ArRobotPeopleBotSH
{
public: 
  AREXPORT ArRobotPeopleBotSH_lms500(const char *dir="");
};

/// @since Aria 2.7.4
class ArRobotPowerBotSH_lms500 : public ArRobotPowerBotSH
{
public: 
  AREXPORT ArRobotPowerBotSH_lms500(const char *dir="");
};

/// @since Aria 2.7.4
class ArRobotResearchPB_lms500 : public ArRobotResearchPB
{
public: 
  AREXPORT ArRobotResearchPB_lms500(const char *dir="");
};

/// @since Aria 2.8
class ArRobotPioneerLX : public ArRobotParams
{
public:
  AREXPORT ArRobotPioneerLX(const char *dir="");
  AREXPORT virtual ~ArRobotPioneerLX() {}
};

/** @endcond INCLUDE_INTERNAL_ROBOT_PARAM_CLASSES */

#endif // ARROBOTTYPES_H