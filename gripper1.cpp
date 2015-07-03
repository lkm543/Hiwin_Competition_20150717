#include "gripper1.h"
////////////////////////////////////////

AmsAddr Addr;
PAmsAddr pAddr=&Addr;	// pointer of Addr;
DWORD dwData;
DWORD dwDatab;
int rpm;
bool GoToPosReqFlag;    //bool PosReqReachedFlag;
int Pos;
BYTE Speed;
BYTE Force;
////////////////////////////////////////
// return valid index
// input 0 or 1 ==> return the same as the input
// otherwise    ==> return 0


////////////////////////////////////////
bool initPara_gripper()
{
	if(!init_gripper_para(0))
		return false;

	return true;
}


bool init_gripper_para()
{
	long nErr,nPort,add0,add1,add2;	// just like hResult for VCI CAN communication
	
	nErr = 0;	// it seems that when it is not 0, it indicates error
	pAddr = &Addr;
	nPort = AdsPortOpen();	// in fact, this var is not used
	nErr = AdsGetLocalAddress(pAddr);

	if(nErr)
	{
		std::cerr << "Error: AdsGetLocalAddress: " << nErr << std::endl;
		return false;
	}
	pAddr->port = 300;	// it is the same for both grippers
	add0=0x11004;

	GoToPosReqFlag = false;

	Pos = 0;
	Speed = 200;
	Force = 100;
}
// end of init_gripper_para
bool initialize_gripper_motion()
{
	long nErr;
	int index;
	


	std::cout<<"initialize gripper ..."<<std::endl;

	long add0,add1,add2;

	add1 = 0x27;	add2 = 0x01;	// byte 0

	dwData = 0x00;
	nErr = AdsSyncWriteReq(pAddr,add0,add1,add2,&dwData);
	if (nErr){
		std:: cerr << "Error: initial_gripper::AdsSyncWriteReq 1 : " << nErr << '\n';
		return false;
		}

	dwData=0x01;
	nErr = 	AdsSyncWriteReq(pAddr,add0,add1,add2,&dwData);
	if (nErr){
		std::cerr << "Error: initial_gripper::AdsSyncWriteReq 2 : " << nErr << '\n';
		return false;
		}

	Sleep(8000);



	return true;
}

bool setGoToPosReqFlag(bool flag)
{
	long nErr;
	long add0,add1,add2;

	add1=0x27; add2=0x1;// byte 0
	dwData = 0x09;

	if(flag == true)
	{
		AdsSyncWriteReq(pAddr,add0,add1,add2,&dwData);
		GoToPosReqFlag = true;
	}
	else{
		// stop
		add1=0x27; add2=0x1;// byte 0

		dwData = 0x01;

		AdsSyncWriteReq(pAddr,add0,add1,add2,&dwData);
		GoToPosReqFlag = false;
		}	// end of else
	return true;
}

bool setReqPos(BYTE reqPos)
{
	if(GoToPosReqFlag == false){
		std::cerr<<"GoToPosReqFlag is false, set it to be true first..."<<std::endl;
		return false;
		}
	long add0,add1,add2,nErr;

	add1=0x2A; add2=0x1;// byte 3
	dwData = reqPos;
	nErr = AdsSyncWriteReq(pAddr,add0,add1,add2,&dwData);
	if (nErr){
		std::cerr << "Error: setReqPos : " << nErr << '\n';
		return false;
		}
	return true;
}

bool setReqSpeed(BYTE reqSpeed)
{
	long add0,add1,add2,nErr;

	add1=0x2B; add2=0x1;// byte 4
	dwData = reqSpeed;
	nErr = AdsSyncWriteReq(pAddr,add0,add1,add2,&dwData);
	if (nErr){
		std::cerr << "Error: setTgtSpeed : " << nErr << '\n';
		return false;
		}
	return true;
}

bool setReqForce( BYTE reqForce)
{
	long add0,add1,add2,nErr;
	
	add1=0x2C; add2=0x1;// byte 5
	dwData = reqForce;
	nErr = AdsSyncWriteReq(pAddr,add0,add1,add2,&dwData);
	if (nErr){
		std::cerr << "Error: setTgtForce : " << nErr << '\n';
		return false;
		}
	return true;
}
