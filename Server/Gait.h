#ifndef GAIT_H
#define GAIT_H

#include <functional>
#include <cstdint>
#include <map>

#include <Aris_ControlData.h>
#include <Aris_Core.h>
#include <Robot_Base.h>
#include <Aris_IMU.h>
#include <Robot_Gait.h>
#include <Robot_Server.h>

struct CALIBRATEG_PARAM :public Robots::GAIT_PARAM_BASE
{
	double distance;
	std::int32_t stayCount;
	std::int32_t moveCount;
};
int calibrateG(Robots::ROBOT_BASE * pRobot, const Robots::GAIT_PARAM_BASE * pParam);
Aris::Core::MSG parseCalibrateG(const std::string &cmd, const std::map<std::string, std::string> &params);

struct MIDMOV_PARAM :public Robots::GAIT_PARAM_BASE
{
	std::int32_t mode;
	double targetDelta[18];
	std::int32_t totalCount;
};
int midmov(Robots::ROBOT_BASE * pRobot, const Robots::GAIT_PARAM_BASE * pParam);
Aris::Core::MSG parseMidmov(const std::string &cmd, const std::map<std::string, std::string> &params);

enum LegState
{
	stance=1,
	swing=2,
	midhang=3,
};

extern Aris::Sensor::IMU imu;

struct PID_PARAM
{
	LegState legState[6];

	std::int32_t count;
	std::int32_t legID;

	const double AccMax=0.8;//m/s^2
	const double VelMax=0.23;//m/s

	double ZEROposeuler[6]{0};
	double IMUposeuler[6]{0};

	double deltaH;
	const double H0=-0.85;

	double pIn_ideal[18];
	double pIn_static[18];
	double pIn_beforePID[18];
	double pIn_afterPID[18];
	double pIn_last[18];
	double pIn_scndlast[18];

	double pEE_ideal[18];//static, without IMUeuler
	double pEE_static[18];//static, with IMUeuler
	double pEE_beforePID[18];
	double pEE_afterPID[18];
	double pEE_beforePID_B[18];
	double pEE_afterPID_B[18];
	double pEE_last_B[18];
	double pEE_scndlast_B[18];

	double pEE_stanceLeg[18];

	double vIn[18];
	double vIn_last[18];
	double aIn[18];

	double aEE_B[18];

	double SafetyDistance[18];

	//For calculating beginPee & beginBodyPE, not used in PID
	double beginPeeofRealParam[18];
	double beginBodyPEofRealParam[6];

};

struct TROT_PARAM :public Robots::GAIT_PARAM_BASE
{
	std::int32_t isIMU;
	std::int32_t totalCount;
	std::int32_t n;
	std::int32_t num;//count for gait

	double distance;//positive in -z direction
	double height;
	double cycleDuty;//0.5 at least
};
int trot(Robots::ROBOT_BASE * pRobot, const Robots::GAIT_PARAM_BASE * pParam);
Aris::Core::MSG parseTrot(const std::string &cmd, const std::map<std::string, std::string> &params);

#endif
