#include <iostream>
#include <cstring>
#include <iomanip> 
#include <bitset>
#include <string>

using namespace std;

#include <stdlib.h>

#include <Platform.h>
#include <Aris_Control.h>
#include <Aris_Message.h>
#include <Robot_Server.h>
#include <Robot_Gait.h>
#include <HexapodIII.h>

#include "Gait.h"

using namespace Aris::Core;

Aris::Sensor::IMU imu;

int test(Robots::ROBOT_BASE * pRobot, const Robots::GAIT_PARAM_BASE * pParam)
{
	auto data = imu.GetSensorData();
	rt_printf("%f  %f  %f\n", data.Get().yaw, data.Get().pitch, data.Get().roll);
	
	auto *p = static_cast<const Robots::WALK_PARAM *>(pParam);
	return p->totalCount - p->count;
}

int main()
{
	imu.Start();

	//Aris::Core::RegisterMsgCallback(1112,OnGetRTMsg);

	auto rs = Robots::ROBOT_SERVER::GetInstance();
	rs->CreateRobot<Robots::ROBOT_III>();

#ifdef PLATFORM_IS_LINUX
	rs->LoadXml("/usr/Robots/resource/HexapodIII/HexapodIII.xml");
#endif
#ifdef PLATFORM_IS_WINDOWS
	rs->LoadXml("C:\\Robots\\resource\\HexapodIII\\HexapodIII.xml");
#endif

	rs->AddGait("wk", test, Robots::parseWalk);
	rs->AddGait("ad", Robots::adjust, Robots::parseAdjust);
	rs->AddGait("cg",calibrateG,parseCalibrateG);
	rs->AddGait("mm",midmov,parseMidmov);
	rs->AddGait("tt",trot,parseTrot);
	rs->Start();



	Aris::Core::RunMsgLoop();

	return 0;
}
