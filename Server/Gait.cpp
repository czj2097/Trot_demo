#include "Gait.h"

#include <Aris_DynKer.h>
#include <Aris_Control.h>
//using namespace Aris::Plan;

//********************************************************************************//

Aris::Core::MSG parseCalibrateG(const std::string &cmd, const std::map<std::string, std::string> &params)
{
	CALIBRATEG_PARAM  param;

	for (auto &i:params)
	{
		if (i.first=="stayCount")
		{
			param.stayCount=stoi(i.second);
		}
		else if (i.first=="moveCount")
		{
			param.moveCount=stoi(i.second);
		}
		else if (i.first=="distance")
		{
			param.distance=stod(i.second);
		}
		else
		{
			std::cout<<"parse failed"<<std::endl;
		}
	}
	Aris::Core::MSG msg;

	msg.CopyStruct(param);

	std::cout<<"finished parse"<<std::endl;

	return msg;
}

int calibrateG(Robots::ROBOT_BASE * pRobot, const Robots::GAIT_PARAM_BASE * pParam)
{
	const CALIBRATEG_PARAM *pCP = static_cast<const CALIBRATEG_PARAM *>(pParam);

	double pEE[18];
	double pBody[6]{0};
	int32_t totalCount;

	memcpy(pEE,pCP->beginPee,sizeof(double)*18);
	totalCount=pCP->stayCount*7+pCP->moveCount*8;

	//move in z direction//
	if(pCP->count<pCP->stayCount)
	{
		//Do nothing,,just stay//
	}
	else if(pCP->count<(pCP->stayCount+pCP->moveCount))
	{
		pBody[2]=-pCP->distance/2*(1-cos(PI*(pCP->count-pCP->stayCount)/pCP->moveCount));
	}
	else if(pCP->count<(2*pCP->stayCount+pCP->moveCount))
	{
		pBody[2]=-pCP->distance;
	}
	else if(pCP->count<(2*pCP->stayCount+3*pCP->moveCount))
	{
		pBody[2]=pCP->distance*(-cos(PI*(pCP->count-2*pCP->stayCount-pCP->moveCount)/(2*pCP->moveCount)));
	}
	else if(pCP->count<(3*pCP->stayCount+3*pCP->moveCount))
	{
		pBody[2]=pCP->distance;
	}
	else if(pCP->count<(3*pCP->stayCount+4*pCP->moveCount))
	{
		pBody[2]=pCP->distance/2*(1+cos(PI*(pCP->count-3*pCP->stayCount-3*pCP->moveCount)/(pCP->moveCount)));
	}

	//move in x direction//
	else if(pCP->count<(4*pCP->stayCount+4*pCP->moveCount))
	{
		//Do nothing,,just stay//
	}
	else if(pCP->count<(4*pCP->stayCount+5*pCP->moveCount))
	{
		pBody[0]=-pCP->distance/2*(1-cos(PI*(pCP->count-4*pCP->stayCount-4*pCP->moveCount)/pCP->moveCount));
	}
	else if(pCP->count<(5*pCP->stayCount+5*pCP->moveCount))
	{
		pBody[0]=-pCP->distance;
	}
	else if(pCP->count<(5*pCP->stayCount+7*pCP->moveCount))
	{
		pBody[0]=pCP->distance*(-cos(PI*(pCP->count-5*pCP->stayCount-5*pCP->moveCount)/(2*pCP->moveCount)));
	}
	else if(pCP->count<(6*pCP->stayCount+7*pCP->moveCount))
	{
		pBody[0]=pCP->distance;
	}
	else if(pCP->count<(6*pCP->stayCount+8*pCP->moveCount))
	{
		pBody[0]=pCP->distance/2*(1+cos(PI*(pCP->count-6*pCP->stayCount-7*pCP->moveCount)/(pCP->moveCount)));
	}
	else if(pCP->count<(7*pCP->stayCount+8*pCP->moveCount))
	{
		//Do nothing,,just stay//
	}

	pRobot->SetPee(pEE,pBody);
	return totalCount-pCP->count-1;
}



//********************************************************************************//

Aris::Core::MSG parseMidmov(const std::string &cmd, const std::map<std::string, std::string> &params)
{
	MIDMOV_PARAM  param;

	for (auto &i:params)
	{
		if (i.first=="x")
		{
			param.targetDelta[3]=-stod(i.second);
			param.targetDelta[12]=stod(i.second);
		}
		else if (i.first=="y")
		{
			param.targetDelta[4]=stod(i.second);
			param.targetDelta[13]=stod(i.second);
		}
		else if (i.first=="z")
		{
			param.targetDelta[5]=stod(i.second);
			param.targetDelta[14]=stod(i.second);
		}
		else if (i.first=="totalCount")
		{
			param.totalCount=stoi(i.second);
		}
		else if(i.first=="mode")
		{
			param.mode=stoi(i.second);
			if(param.mode!=0&&param.mode!=1)
				std::cout<<"mode input wrong!"<<std::endl;
		}
		else
		{
			std::cout<<"parse failed"<<std::endl;
		}
	}
	Aris::Core::MSG msg;

	msg.CopyStruct(param);

	std::cout<<"finished parse"<<std::endl;

	return msg;
}

int midmov(Robots::ROBOT_BASE * pRobot, const Robots::GAIT_PARAM_BASE * pParam)
{
	const MIDMOV_PARAM *pMP = static_cast<const MIDMOV_PARAM *>(pParam);
	double pEE[18];
	double pBody[6];
	memcpy(pBody,pMP->beginBodyPE,sizeof(double)*6);
	for (int i=0;i<18;i++)
	{
		if(pMP->mode==0)
			pEE[i]=pMP->targetDelta[i]/2*(1-cos(PI*pMP->count/pMP->totalCount))+pMP->beginPee[i];
		else if(pMP->mode==1)
			pEE[i]=-pMP->targetDelta[i]/2*(1-cos(PI*pMP->count/pMP->totalCount))+pMP->beginPee[i];
	}

	pRobot->SetPee(pEE,pBody);
	return pMP->totalCount-pMP->count-1;
}




//********************************************************************************//

Aris::Core::MSG parseTrot(const std::string &cmd, const std::map<std::string, std::string> &params)
{
	TROT_PARAM  param;

	for (auto &i:params)
	{
		if (i.first=="imu")
		{
			param.isIMU=stoi(i.second);
		}
		else if (i.first=="totalCount")
		{
			param.totalCount=stoi(i.second);
		}
		else if (i.first=="n")
		{
			param.n=stoi(i.second);
		}
		else if(i.first=="distance")
		{
			param.distance=stod(i.second);
			if (param.distance>0.4)
				param.distance=0.4;
		}
		else if(i.first=="height")
		{
			param.height=stod(i.second);
		}
		else if(i.first=="cycleDuty")
		{
			param.cycleDuty=stod(i.second);
			if (param.cycleDuty<0.5)
				param.cycleDuty=0.5;
		}
		else
		{
			std::cout<<"parse failed"<<std::endl;
		}
	}
	Aris::Core::MSG msg;

	msg.CopyStruct(param);

	std::cout<<"finished parse"<<std::endl;

	return msg;
}

void calculateHeight(PID_PARAM * pPID , Robots::ROBOT_BASE * pRobot)
{
	/***********CALCULATE DELTAH***********/
	double heightSum{0};
	double heightBodyAvg;
	std::int32_t sum{0};
	pRobot->GetPin(pPID->pIn_static);
	pRobot->SetPin(pPID->pIn_static,pPID->IMUposeuler);
	for(int i=0;i<6;i++)
	{
		pRobot->pLegs[i]->GetPee(pPID->pEE_static+3*i);
		if (pPID->legState[i]==LegState::stance)
		{
			sum++;
			heightSum+=pPID->pEE_static[3*i+1];
		}
	}
	heightBodyAvg=heightSum/sum;

	pPID->deltaH=heightBodyAvg-pPID->H0;//positive +
}

void setpInLimit(PID_PARAM * pPID)
{
	/********SET POSITION LIMIT HERE************/
	//decelerate at AccMax when getting screwpos safety distance
	int i=pPID->legID;

	for (int j=0;j<3;j++)
	{
		pPID->SafetyDistance[3*i+j]=0.5*pPID->vIn_last[3*i+j]*pPID->vIn_last[3*i+j]/pPID->AccMax;
	}

	if (pPID->pIn_afterPID[3*i] >= 1.091 - pPID->SafetyDistance[3*i])
	{
		pPID->pIn_afterPID[3*i]=pPID->pIn_last[3*i]+pPID->vIn_last[3*i]*0.001-pPID->AccMax/2*0.001*0.001;
		pPID->vIn[3*i]=pPID->vIn_last[3*i]-pPID->AccMax*0.001;
	}
	else if(pPID->pIn_afterPID[3*i] <= 0.68 + pPID->SafetyDistance[3*i])
	{
		pPID->pIn_afterPID[3*i]=pPID->pIn_last[3*i]+pPID->vIn_last[3*i]*0.001+pPID->AccMax/2*0.001*0.001;
		pPID->vIn[3*i]=pPID->vIn_last[3*i]+pPID->AccMax*0.001;
	}

	if (pPID->pIn_afterPID[3*i+1] >= 1.112 - pPID->SafetyDistance[3*i+1])
	{
		pPID->pIn_afterPID[3*i+1]=pPID->pIn_last[3*i+1]+pPID->vIn_last[3*i+1]*0.001-pPID->AccMax/2*0.001*0.001;
		pPID->vIn[3*i+1]=pPID->vIn_last[3*i+1]-pPID->AccMax*0.001;
	}
	else if(pPID->pIn_afterPID[3*i+1] <= 0.702 + pPID->SafetyDistance[3*i+1])
	{
		pPID->pIn_afterPID[3*i+1]=pPID->pIn_last[3*i+1]+pPID->vIn_last[3*i+1]*0.001+pPID->AccMax/2*0.001*0.001;
		pPID->vIn[3*i+1]=pPID->vIn_last[3*i+1]+pPID->AccMax*0.001;
	}

	if (pPID->pIn_afterPID[3*i+2] >= 1.112 - pPID->SafetyDistance[3*i+2])
	{
		pPID->pIn_afterPID[3*i+2]=pPID->pIn_last[3*i+2]+pPID->vIn_last[3*i+2]*0.001-pPID->AccMax/2*0.001*0.001;
		pPID->vIn[3*i+2]=pPID->vIn_last[3*i+2]-pPID->AccMax*0.001;
	}
	else if(pPID->pIn_afterPID[3*i+2] <= 0.702 + pPID->SafetyDistance[3*i+2])
	{
		pPID->pIn_afterPID[3*i+2]=pPID->pIn_last[3*i+2]+pPID->vIn_last[3*i+2]*0.001+pPID->AccMax/2*0.001*0.001;
		pPID->vIn[3*i+2]=pPID->vIn_last[3*i+2]+pPID->AccMax*0.001;
	}
}

//**********************DOPID2: PIN ERROR AS INPUT, VIN AS OUTPUT*******************************//
void doPID1(PID_PARAM * pPID , Robots::ROBOT_BASE * pRobot)
{
	calculateHeight(pPID,pRobot);

	if(pPID->count==0)
	{
		memcpy(pPID->pIn_last,pPID->pIn_static,sizeof(double)*18);
		memcpy(pPID->pIn_scndlast,pPID->pIn_last,sizeof(double)*18);

		memcpy(pPID->pEE_stanceLeg,pPID->pEE_ideal,sizeof(double)*18);
	}

	for (int i=0;i<6;i++)
	{
		pPID->legID=i;

		double Kp_stance{10};
		double Kd_stance{0};
		double Ki_stance{0};
		double Kp_swing={26};
		double Kd_swing{0};
		double Ki_swing{0};
		switch(pPID->legState[i])
		{
		case LegState::stance:
			memcpy(pPID->pEE_beforePID+3*i,pPID->pEE_stanceLeg+3*i,sizeof(double)*3);
			pPID->pEE_beforePID[3*i+1]=pPID->H0-pPID->deltaH;

			pRobot->SetPee(pPID->pEE_beforePID,pPID->ZEROposeuler);
			pRobot->pLegs[i]->GetPin(pPID->pIn_beforePID+3*i);

			for(int j=0;j<3;j++)
			{
				pPID->vIn[3*i+j]=Kp_stance*(pPID->pIn_beforePID[3*i+j]-pPID->pIn_last[3*i+j])+
								Kd_stance*(pPID->pIn_beforePID[3*i+j]+pPID->pIn_scndlast[3*i+j]-2*pPID->pIn_last[3*i+j])+
								Ki_stance*pPID->pIn_beforePID[3*i+j];

				if(pPID->vIn[3*i+j]-pPID->vIn_last[3*i+j]>pPID->AccMax*0.001)
					pPID->vIn[3*i+j]=pPID->vIn_last[3*i+j]+pPID->AccMax*0.001;
				else if(pPID->vIn[3*i+j]-pPID->vIn_last[3*i+j]<-pPID->AccMax*0.001)
					pPID->vIn[3*i+j]=pPID->vIn_last[3*i+j]-pPID->AccMax*0.001;

				if(pPID->vIn[3*i+j]>pPID->VelMax)
					pPID->vIn[3*i+j]=pPID->VelMax;
				else if(pPID->vIn[3*i+j]<-pPID->VelMax)
					pPID->vIn[3*i+j]=-pPID->VelMax;

				pPID->pIn_afterPID[3*i+j]=pPID->pIn_last[3*i+j]+pPID->vIn[3*i+j]/1000;
			}

			setpInLimit(pPID);

			break;

		case LegState::swing:
			memcpy(pPID->pEE_beforePID+3*i,pPID->pEE_ideal+3*i,sizeof(double)*3);
			pPID->pEE_beforePID[3*i+1]=pPID->pEE_ideal[3*i+1]+pPID->deltaH;//x,z stay the same with static traj
/*
			//CGait::online_trot_TargetXZ(IMU_angleVel,online_body_height,i,TargetXZ);
			CGait::online_trot_TargetXZ(online_angleVel,online_body_height,i,TargetXZ);
			online_ideal_foot_pos[3*i]=online_static_foot_pos[3*i]+TargetXZ[0];
			online_ideal_foot_pos[3*i+2]=online_static_foot_pos[3*i+2]+TargetXZ[1];
*/

			pRobot->SetPee(pPID->pEE_beforePID,pPID->IMUposeuler);
			pRobot->pLegs[i]->GetPin(pPID->pIn_beforePID+3*i);

			for(int j=0;j<3;j++)
			{
				pPID->vIn[3*i+j]=Kp_swing*(pPID->pIn_beforePID[3*i+j]-pPID->pIn_last[3*i+j])+
								Kd_swing*(pPID->pIn_beforePID[3*i+j]+pPID->pIn_scndlast[3*i+j]-2*pPID->pIn_last[3*i+j])+
								Ki_swing*pPID->pIn_beforePID[3*i+j];

				if(pPID->vIn[3*i+j]-pPID->vIn_last[3*i+j]>pPID->AccMax*0.001)
					pPID->vIn[3*i+j]=pPID->vIn_last[3*i+j]+pPID->AccMax*0.001;
				else if(pPID->vIn[3*i+j]-pPID->vIn_last[3*i+j]<-pPID->AccMax*0.001)
					pPID->vIn[3*i+j]=pPID->vIn_last[3*i+j]-pPID->AccMax*0.001;

				if(pPID->vIn[3*i+j]>pPID->VelMax)
					pPID->vIn[3*i+j]=pPID->VelMax;
				else if(pPID->vIn[3*i+j]<-pPID->VelMax)
					pPID->vIn[3*i+j]=-pPID->VelMax;

				pPID->pIn_afterPID[3*i+j]=pPID->pIn_last[3*i+j]+pPID->vIn[3*i+j]/1000;
			}

			setpInLimit(pPID);

			pRobot->pLegs[i]->SetPin(pPID->pIn_afterPID+3*i);
			pRobot->pLegs[i]->GetPee(pPID->pEE_afterPID+3*i);

			memcpy(pPID->pEE_stanceLeg+3*i,pPID->pEE_afterPID+3*i,sizeof(double)*3);
			//stance leg pos is refreshed every ms, but only used when it transfers to stance leg

			break;

		case LegState::midhang:
			memcpy(pPID->pIn_afterPID+3*i,pPID->pIn_static+3*i,sizeof(double)*3);
			break;
		default:
			break;
		}
	}
}


//**********************DOPID2: PEE ERROR AS INPUT, AEE AS OUTPUT*******************************//
void setaInLimit(PID_PARAM * pPID , Robots::ROBOT_BASE * pRobot)
{
	int i=pPID->legID;

	double Jvi_B[3][3];
	double DifJvd_B[3][3];

	double Cad[3][1]{0};
	double aEE_B[3][1]{0};

	double vIn_last[3][1];

	double alpha{1};
	double beta{1};

	pRobot->pLegs[i]->GetJvi(*Jvi_B,"B");
	pRobot->pLegs[i]->GetDifJvd(*DifJvd_B,"B");

	memcpy(*vIn_last,pPID->vIn_last+3*i,sizeof(double)*3);

	memcpy(*aEE_B,pPID->aEE_B+3*i,sizeof(double)*3);
	Aris::DynKer::s_dgemm(3,1,3,-alpha,*DifJvd_B,3,*vIn_last,1,beta,*aEE_B,1);
	Aris::DynKer::s_dgemm(3,1,3,alpha,*Jvi_B,3,*aEE_B,1,beta,*Cad,1);

	for(int j=0;j<3;j++)
	{
		if(Cad[j][0]>pPID->AccMax)
			pPID->aIn[3*i+j]=pPID->AccMax;
		else if(Cad[j][0]<-pPID->AccMax)
			pPID->aIn[3*i+j]=-pPID->AccMax;
		else
			pPID->aIn[3*i+j]=Cad[j][0];
	}
}

void setvInLimit(PID_PARAM * pPID , Robots::ROBOT_BASE * pRobot)
{
	int i=pPID->legID;

	//calculating direction of moving
	double DirEE_B[3][1]{0};//dirction of vEE
	double Jvi_B[3][3];
	double DirIn[3][1]{0};//dirction of vIn
	pRobot->pLegs[i]->GetJvi(*Jvi_B,"B");
	for (int j=0;j<3;j++)
		DirEE_B[j][0]=pPID->pEE_beforePID_B[3*i+j]-pPID->pEE_last_B[3*i+j];
	Aris::DynKer::s_dgemm(3,1,3,1,*Jvi_B,3,*DirEE_B,1,1,*DirIn,1);

	//project the vIn vector to the moving direction
	double vIn[3];
	double lengthvIn{0};
	double lengthDir{0};
	for (int j=0;j<3;j++)
	{
		vIn[j]=pPID->aIn[3*i+j]+pPID->vIn_last[3*i+j];
		lengthvIn=lengthvIn+vIn[j]*DirIn[j][0];
		lengthDir=lengthDir+DirIn[j][0]*DirIn[j][0];
	}
	for (int j=0;j<3;j++)
		vIn[j]=lengthvIn*DirIn[j][0]/lengthDir;

	//set limit to vIn
	double vInMax{0};
	double Kv{1};
	for (int j=0;j<3;j++)
	{
		if(vInMax<=std::abs(vIn[j]))
			vInMax=std::abs(vIn[j]);
	}
	if(vInMax>pPID->VelMax)
		Kv=pPID->VelMax/vInMax;//between 0 & 1
	for(int j=0;j<3;j++)
		pPID->vIn[3*i+j]=Kv*vIn[j];
}

void doPID2(PID_PARAM * pPID , Robots::ROBOT_BASE * pRobot)
{
	calculateHeight(pPID,pRobot);

	if(pPID->count==0)
	{
		memcpy(pPID->pEE_last_B,pPID->pEE_static,sizeof(double)*18);
		memcpy(pPID->pEE_scndlast_B,pPID->pEE_last_B,sizeof(double)*18);

		memcpy(pPID->pEE_stanceLeg,pPID->pEE_ideal,sizeof(double)*18);
	}

	for(int i=0;i<6;i++)
	{
		pPID->legID=i;

		double Kp_stance{10};
		double Kd_stance{0};
		double Ki_stance{0};
		double Kp_swing={26};
		double Kd_swing{0};
		double Ki_swing{0};
		switch(pPID->legState[i])
		{
		case LegState::stance:
			memcpy(pPID->pEE_beforePID+3*i,pPID->pEE_stanceLeg+3*i,sizeof(double)*3);
			pPID->pEE_beforePID[3*i+1]=pPID->H0-pPID->deltaH;

			pRobot->SetPee(pPID->pEE_beforePID,pPID->ZEROposeuler);
			pRobot->pLegs[i]->GetPin(pPID->pIn_beforePID+3*i);

			pRobot->pLegs[i]->GetPee(pPID->pEE_beforePID_B+3*i,"B");

			for(int j=0;j<3;j++)
			{
				pPID->aEE_B[3*i+j]=Kp_stance*(pPID->pEE_beforePID_B[3*i+j]-pPID->pEE_last_B[3*i+j])+
								Kd_stance*(pPID->pEE_beforePID_B[3*i+j]+pPID->pEE_scndlast_B[3*i+j]-2*pPID->pEE_last_B[3*i+j])+
								Ki_stance*pPID->pEE_beforePID_B[3*i+j];
			}

			setaInLimit(pPID,pRobot);
			setvInLimit(pPID,pRobot);

			for(int j=0;j<3;j++)
				pPID->pIn_afterPID[3*i+j]=pPID->pIn_last[3*i+j]+pPID->vIn[3*i+j]/1000;

			setpInLimit(pPID);

			break;

		case LegState::swing:
			memcpy(pPID->pEE_beforePID+3*i,pPID->pEE_ideal+3*i,sizeof(double)*3);
			pPID->pEE_beforePID[3*i+1]=pPID->pEE_ideal[3*i+1]+pPID->deltaH;

			pRobot->SetPee(pPID->pEE_beforePID,pPID->IMUposeuler);

			pRobot->pLegs[i]->GetPee(pPID->pEE_beforePID_B+3*i,"B");

			for(int j=0;j<3;j++)
			{
				pPID->aEE_B[3*i+j]=Kp_stance*(pPID->pEE_beforePID_B[3*i+j]-pPID->pEE_last_B[3*i+j])+
								Kd_stance*(pPID->pEE_beforePID_B[3*i+j]+pPID->pEE_scndlast_B[3*i+j]-2*pPID->pEE_last_B[3*i+j])+
								Ki_stance*pPID->pEE_beforePID_B[3*i+j];
			}

			setaInLimit(pPID,pRobot);
			setvInLimit(pPID,pRobot);

			for(int j=0;j<3;j++)
				pPID->pIn_afterPID[3*i+j]=pPID->pIn_last[3*i+j]+pPID->vIn[3*i+j]/1000;

			setpInLimit(pPID);

			pRobot->pLegs[i]->SetPin(pPID->pIn_afterPID+3*i);
			pRobot->pLegs[i]->GetPee(pPID->pEE_afterPID+3*i);

			memcpy(pPID->pEE_stanceLeg+3*i,pPID->pEE_afterPID+3*i,sizeof(double)*3);

			break;

		case LegState::midhang:
			memcpy(pPID->pIn_afterPID+3*i,pPID->pIn_static+3*i,sizeof(double)*3);
			break;
		default:
			break;
		}
	}
}


int trotAcc(Robots::ROBOT_BASE * pRobot, const TROT_PARAM * pParam, PID_PARAM * pPID)
{
	const TROT_PARAM *pTrotParam = static_cast<const TROT_PARAM *>(pParam);

	double delta[3]{0};
	double targetDeltaEE[18]{0};
	double targetDeltaBody[6]{0};
	double count=pTrotParam->count;//change data type to realize dividing

	delta[1]=pTrotParam->height;
	delta[2]=-pTrotParam->distance/2;

	double pEE[18];
	double pBody[6];

	double swingCount=pTrotParam->totalCount*(1-pTrotParam->cycleDuty);//little than totalCount/2

	//calculate pEE, including midhang legs. move leg 0 & 5
	memcpy(targetDeltaEE,delta,sizeof(double)*3);
	memcpy(targetDeltaEE+3*5,delta,sizeof(double)*3);
	if (pTrotParam->count<swingCount)
	{
		for (int i=0;i<18;i++)
		{
			if (i%3==1)//y
				pEE[i]=targetDeltaEE[i]/2*(1-cos(2*PI*count/swingCount))+pTrotParam->beginPee[i];
			else//x&z
				pEE[i]=targetDeltaEE[i]/2*(1-cos(PI*count/swingCount))+pTrotParam->beginPee[i];
		}
		if (pTrotParam->isIMU!=0)
		{
			pPID->legState[0]=LegState::swing;
			pPID->legState[5]=LegState::swing;
			pPID->legState[2]=LegState::stance;
			pPID->legState[3]=LegState::stance;
			pPID->legState[1]=LegState::midhang;
			pPID->legState[4]=LegState::midhang;
		}
	}
	else//end effectors stay still
	{
		for (int i=0;i<18;i++)
		{
			if (i%3==1)//y
				pEE[i]=pTrotParam->beginPee[i];
			else//x&z
				pEE[i]=targetDeltaEE[i]+pTrotParam->beginPee[i];
		}
		if (pTrotParam->isIMU!=0)
		{
			pPID->legState[0]=LegState::stance;
			pPID->legState[5]=LegState::stance;
			pPID->legState[2]=LegState::stance;
			pPID->legState[3]=LegState::stance;
			pPID->legState[1]=LegState::midhang;
			pPID->legState[4]=LegState::midhang;
		}
	}

	//calculate pBody
	targetDeltaBody[2]=-pTrotParam->distance/4;
	for (int i=0;i<6;i++)
	{
		pBody[i]=targetDeltaBody[i]*4*(count/pTrotParam->totalCount)*(count/pTrotParam->totalCount)+pTrotParam->beginBodyPE[i];
	}

	//correct midhang legs, in z direction
	pEE[5]=pBody[2];
	pEE[14]=pBody[2];

	pRobot->SetPee(pEE,pBody);

	//with IMU data
	if(pTrotParam->isIMU!=0)
	{
		memcpy(pPID->pEE_ideal,pEE,sizeof(double)*18);
		memcpy(pPID->ZEROposeuler,pBody,sizeof(double)*6);
		memcpy(pPID->IMUposeuler,pBody,sizeof(double)*3);

		auto data = imu.GetSensorData();
		data.Get().ToBodyEul(pPID->IMUposeuler+3,PI);

		if(pTrotParam->isIMU==1)
		{
			doPID1(pPID,pRobot);
			pRobot->SetPin(pPID->pIn_afterPID,pPID->IMUposeuler);

			pRobot->GetPee(pPID->pEE_afterPID);//for data output

			memcpy(pPID->pIn_scndlast,pPID->pIn_last,sizeof(double)*18);
			memcpy(pPID->pIn_last,pPID->pIn_afterPID,sizeof(double)*18);
		}

		else if(pTrotParam->isIMU==2)
		{
			doPID2(pPID,pRobot);
			pRobot->SetPin(pPID->pIn_afterPID,pPID->IMUposeuler);

			pRobot->GetPee(pPID->pEE_afterPID_B,"B");

			memcpy(pPID->pEE_scndlast_B,pPID->pEE_last_B,sizeof(double)*18);
			memcpy(pPID->pEE_last_B,pPID->pEE_afterPID,sizeof(double)*18);
		}

		memcpy(pPID->vIn_last,pPID->vIn,sizeof(double)*18);
	}

	return pTrotParam->totalCount/2-pTrotParam->count-1;
}

int trotConst(Robots::ROBOT_BASE * pRobot, const TROT_PARAM * pParam, PID_PARAM * pPID)
{
	const TROT_PARAM *pTrotParam = static_cast<const TROT_PARAM *>(pParam);

	double delta[3]{0};
	double targetDeltaEE[18]{0};
	double targetDeltaBody[6]{0};
	double count=pTrotParam->count;//change data type to realize dividing

	delta[1]=pTrotParam->height;
	delta[2]=-pTrotParam->distance;

	double pEE[18];
	double pBody[18];

	double swingCount=pTrotParam->totalCount*(1-pTrotParam->cycleDuty);

	int iter=(2*pTrotParam->count/pTrotParam->totalCount)%2;
	if(iter==0)//iter==0,first half cycle,move leg 2 & 3
	{
		memcpy(targetDeltaEE+3*2,delta,sizeof(double)*3);
		memcpy(targetDeltaEE+3*3,delta,sizeof(double)*3);

		if (pTrotParam->count<swingCount)
		{
			for(int i=0;i<18;i++)
			{
				if (i%3==1)//y
					pEE[i]=targetDeltaEE[i]/2*(1-cos(2*PI*count/swingCount))+pTrotParam->beginPee[i];
				else//x&z
					pEE[i]=targetDeltaEE[i]/2*(1-cos(PI*count/swingCount))+pTrotParam->beginPee[i];
			}
			if (pTrotParam->isIMU!=0)
			{
				pPID->legState[0]=LegState::stance;
				pPID->legState[5]=LegState::stance;
				pPID->legState[2]=LegState::swing;
				pPID->legState[3]=LegState::swing;
				pPID->legState[1]=LegState::midhang;
				pPID->legState[4]=LegState::midhang;
			}
		}
		else//end effectors stay still
		{
			for(int i=0;i<18;i++)
			{
				if (i%3==1)//y
					pEE[i]=pTrotParam->beginPee[i];
				else//x&z
					pEE[i]=targetDeltaEE[i]+pTrotParam->beginPee[i];
			}
			if (pTrotParam->isIMU!=0)
			{
				pPID->legState[0]=LegState::stance;
				pPID->legState[5]=LegState::stance;
				pPID->legState[2]=LegState::stance;
				pPID->legState[3]=LegState::stance;
				pPID->legState[1]=LegState::midhang;
				pPID->legState[4]=LegState::midhang;
			}
		}
	}
	else//iter=1,second half cycle,move leg 0 & 5
	{
		memcpy(targetDeltaEE+3*0,delta,sizeof(double)*3);
		memcpy(targetDeltaEE+3*5,delta,sizeof(double)*3);

		double beginPee[18];
		memcpy(beginPee,pTrotParam->beginPee,sizeof(double)*18);
		beginPee[3*2+2]=pTrotParam->beginPee[3*2+2]-pTrotParam->distance;
		beginPee[3*3+2]=pTrotParam->beginPee[3*3+2]-pTrotParam->distance;

		if ((pTrotParam->count-pTrotParam->totalCount/2)<swingCount)//end effectors swing
		{
			for(int i=0;i<18;i++)
			{
				if (i%3==1)//y
					pEE[i]=targetDeltaEE[i]/2*(1-cos(2*PI*(count-pTrotParam->totalCount/2)/swingCount))+beginPee[i];
				else//x&z
					pEE[i]=targetDeltaEE[i]/2*(1-cos(PI*(count-pTrotParam->totalCount/2)/swingCount))+beginPee[i];
			}
			if (pTrotParam->isIMU!=0)
			{
				pPID->legState[0]=LegState::swing;
				pPID->legState[5]=LegState::swing;
				pPID->legState[2]=LegState::stance;
				pPID->legState[3]=LegState::stance;
				pPID->legState[1]=LegState::midhang;
				pPID->legState[4]=LegState::midhang;
			}
		}
		else//end effectors stay still
		{
			for(int i=0;i<18;i++)
			{
				if (i%3==1)//y
					pEE[i]=pTrotParam->beginPee[i];
				else//x&z
					pEE[i]=targetDeltaEE[i]+beginPee[i];
			}
			if (pTrotParam->isIMU!=0)
			{
				pPID->legState[0]=LegState::stance;
				pPID->legState[5]=LegState::stance;
				pPID->legState[2]=LegState::stance;
				pPID->legState[3]=LegState::stance;
				pPID->legState[1]=LegState::midhang;
				pPID->legState[4]=LegState::midhang;
			}
		}
	}

	//calculate pBody
	targetDeltaBody[2]=-pTrotParam->distance;
	for (int i=0;i<6;i++)
	{
		pBody[i]=targetDeltaBody[i]*count/pTrotParam->totalCount+pTrotParam->beginBodyPE[i];
	}

	//correct midhang legs, in z direction
	pEE[5]=pBody[2];
	pEE[14]=pBody[2];

	pRobot->SetPee(pEE,pBody);

	//with IMU data
	if(pTrotParam->isIMU!=0)
	{
		memcpy(pPID->pEE_ideal,pEE,sizeof(double)*18);
		memcpy(pPID->ZEROposeuler,pBody,sizeof(double)*6);
		memcpy(pPID->IMUposeuler,pBody,sizeof(double)*3);

		auto data = imu.GetSensorData();
		data.Get().ToBodyEul(pPID->IMUposeuler+3,PI);

		if(pTrotParam->isIMU==1)
		{
			doPID1(pPID,pRobot);
			pRobot->SetPin(pPID->pIn_afterPID,pPID->IMUposeuler);

			pRobot->GetPee(pPID->pEE_afterPID);//only for data output

			memcpy(pPID->pIn_scndlast,pPID->pIn_last,sizeof(double)*18);
			memcpy(pPID->pIn_last,pPID->pIn_afterPID,sizeof(double)*18);
		}

		else if(pTrotParam->isIMU==2)
		{
			doPID2(pPID,pRobot);
			pRobot->SetPin(pPID->pIn_afterPID,pPID->IMUposeuler);

			pRobot->GetPee(pPID->pEE_afterPID_B,"B");

			memcpy(pPID->pEE_scndlast_B,pPID->pEE_last_B,sizeof(double)*18);
			memcpy(pPID->pEE_last_B,pPID->pEE_afterPID,sizeof(double)*18);
		}

		memcpy(pPID->vIn_last,pPID->vIn,sizeof(double)*18);
	}

	return pTrotParam->totalCount-pTrotParam->count-1;
}

int trotDec(Robots::ROBOT_BASE * pRobot, const TROT_PARAM * pParam, PID_PARAM * pPID)
{
	const TROT_PARAM *pTrotParam = static_cast<const TROT_PARAM *>(pParam);

	double delta[3]{0};
	double targetDeltaEE[18]{0};
	double targetDeltaBody[6]{0};
	double count=pTrotParam->count;//change data type to realize dividing

	delta[1]=pTrotParam->height;
	delta[2]=-pTrotParam->distance/2;

	double pEE[18];
	double pBody[6];

	double swingCount=pTrotParam->totalCount*(1-pTrotParam->cycleDuty);//little than totalCount/2

	//calculate pEE. move leg 2 & 3
	memcpy(targetDeltaEE+3*2,delta,sizeof(double)*3);
	memcpy(targetDeltaEE+3*3,delta,sizeof(double)*3);

	if (pTrotParam->count<swingCount)
	{
		for (int i=0;i<18;i++)
		{
			if (i%3==1)//y
				pEE[i]=targetDeltaEE[i]/2*(1-cos(2*PI*count/swingCount))+pTrotParam->beginPee[i];
			else//x&z
				pEE[i]=targetDeltaEE[i]/2*(1-cos(PI*count/swingCount))+pTrotParam->beginPee[i];
		}
		if (pTrotParam->isIMU!=0)
		{
			pPID->legState[0]=LegState::stance;
			pPID->legState[5]=LegState::stance;
			pPID->legState[2]=LegState::swing;
			pPID->legState[3]=LegState::swing;
			pPID->legState[1]=LegState::midhang;
			pPID->legState[4]=LegState::midhang;
		}
	}
	else//end effectors stay still
	{
		for (int i=0;i<18;i++)
		{
			if (i%3==1)//y
				pEE[i]=pTrotParam->beginPee[i];
			else//x&z
				pEE[i]=targetDeltaEE[i]+pTrotParam->beginPee[i];
		}
		if (pTrotParam->isIMU!=0)
		{
			pPID->legState[0]=LegState::stance;
			pPID->legState[5]=LegState::stance;
			pPID->legState[2]=LegState::stance;
			pPID->legState[3]=LegState::stance;
			pPID->legState[1]=LegState::midhang;
			pPID->legState[4]=LegState::midhang;
		}
	}

	//calculate pBody
	targetDeltaBody[2]=-pTrotParam->distance/4;
	for (int i=0;i<6;i++)
	{
		pBody[i]=targetDeltaBody[i]*(1-(2*count/pTrotParam->totalCount-1)*(2*count/pTrotParam->totalCount-1))+pTrotParam->beginBodyPE[i];
	}

	//correct midhang legs, in z direction
	pEE[5]=pBody[2];
	pEE[14]=pBody[2];

	pRobot->SetPee(pEE,pBody);

	//with IMU data
	if(pTrotParam->isIMU!=0)
	{
		memcpy(pPID->pEE_ideal,pEE,sizeof(double)*18);
		memcpy(pPID->ZEROposeuler,pBody,sizeof(double)*6);
		memcpy(pPID->IMUposeuler,pBody,sizeof(double)*3);

		auto data = imu.GetSensorData();
		data.Get().ToBodyEul(pPID->IMUposeuler+3,PI);

		if(pTrotParam->isIMU==1)
		{
			doPID1(pPID,pRobot);
			pRobot->SetPin(pPID->pIn_afterPID,pPID->IMUposeuler);

			pRobot->GetPee(pPID->pEE_afterPID);//only for data output

			memcpy(pPID->pIn_scndlast,pPID->pIn_last,sizeof(double)*18);
			memcpy(pPID->pIn_last,pPID->pIn_afterPID,sizeof(double)*18);
		}

		else if(pTrotParam->isIMU==2)
		{
			doPID2(pPID,pRobot);
			pRobot->SetPin(pPID->pIn_afterPID,pPID->IMUposeuler);

			pRobot->GetPee(pPID->pEE_afterPID_B,"B");

			memcpy(pPID->pEE_scndlast_B,pPID->pEE_last_B,sizeof(double)*18);
			memcpy(pPID->pEE_last_B,pPID->pEE_afterPID,sizeof(double)*18);
		}

		memcpy(pPID->vIn_last,pPID->vIn,sizeof(double)*18);
	}

	return pTrotParam->totalCount/2-pTrotParam->count-1;
}

int trot(Robots::ROBOT_BASE * pRobot, const Robots::GAIT_PARAM_BASE * pParam)
{
	const TROT_PARAM *pTP = static_cast<const TROT_PARAM *>(pParam);

	static PID_PARAM pPID;

	TROT_PARAM realParam = *pTP;

	pPID.count=pTP->count;

	realParam.count = (pTP->count - pTP->totalCount/2) % pTP->totalCount;
	realParam.num=(pTP->count - pTP->totalCount/2) / pTP->totalCount;

	realParam.beginPee[3*0+2]=pTP->beginPee[3*0+2]-(realParam.distance/2+realParam.num*realParam.distance);
	realParam.beginPee[3*5+2]=pTP->beginPee[3*5+2]-(realParam.distance/2+realParam.num*realParam.distance);

	realParam.beginPee[3*2+2]=pTP->beginPee[3*2+2]-realParam.num*realParam.distance;
	realParam.beginPee[3*3+2]=pTP->beginPee[3*3+2]-realParam.num*realParam.distance;

	realParam.beginPee[3*1+2]=pTP->beginPee[3*1+2]-(realParam.distance/4+realParam.num*realParam.distance);
	realParam.beginPee[3*4+2]=pTP->beginPee[3*4+2]-(realParam.distance/4+realParam.num*realParam.distance);
	realParam.beginBodyPE[2]=pTP->beginBodyPE[2]-(realParam.distance/4+realParam.num*realParam.distance);

	if (pParam->count < pTP->totalCount/2)
	{
		trotAcc(pRobot, pTP, &pPID);
	}
	else if (pParam->count < (pTP->n*2+1) * pTP->totalCount/2)
	{
		trotConst(pRobot, &realParam, &pPID);
	}
	else
	{
		trotDec(pRobot, &realParam, &pPID);
	}

	Robots::ROBOT_SERVER::GetInstance()->GetCs();

	return (pTP->n+1) * pTP->totalCount - pTP->count - 1;

}
