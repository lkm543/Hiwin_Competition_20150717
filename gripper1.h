#ifndef _GRIPPER_
#define _GRIPPER_

//#include "robot.h"		// including some constants of both grippers
#include <process.h>	// to use _beginthread
#include <iostream>
#include <windows.h>
#include "C:\Twincat\AdsApi\TcAdsDll\Include\TcAdsDef.h"
#include "C:\Twincat\AdsApi\TcAdsDll\Include\TcAdsApi.h"

using namespace std;
///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
//	// nErr is executed status of functions
//	// nPort is not used, but the line that nPort is used as left-value should be reserved
//	// 
//	// add1 is IOffs shown in TwinCAT manager, which is different for each byte. 
//	// add2 is the Len shown in TwinCAT manager, which is the length of each byte, 1.
//extern long nErr[2];
//extern long nPort[ NUMBER_OF_GRIPPER ];
//
//extern AmsAddr Addr[ NUMBER_OF_GRIPPER ];
//extern PAmsAddr pAddr[ NUMBER_OF_GRIPPER ];	// pointer of Addr;
//
//extern long Rxadd0[ NUMBER_OF_GRIPPER ];
//extern long Txadd0[ NUMBER_OF_GRIPPER ];

extern DWORD dwData;
extern DWORD dwDatab;

//extern BYTE GripperMode[ NUMBER_OF_GRIPPER ];		// indicates which mode the gripper is in, only meaningful for 3-fingered gripper
//extern bool GoToPosReqFlag[ NUMBER_OF_GRIPPER ];
////extern bool PosReqReachedFlag;
//
//extern int Pos[ NUMBER_OF_GRIPPER ];
//extern BYTE Speed[ NUMBER_OF_GRIPPER ];
//extern BYTE Force[ NUMBER_OF_GRIPPER ];


///////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
// util function
int checkGripperIdx(int gripperIdx);

// use structIdx to discriminate between 3-fingered and 2-fingered
// 0 = 2-fingered 1 = 3-fingered
// just the same as armIdx
bool initPara_gripper();

bool init_gripper_para(int gripperIdx);
bool initialize_gripper_motion(int gripperIdx);

// en/di function of the gripper
bool setGoToPosReqFlag(bool flag , int gripperIdx);

//BYTE ranges from 0 to 255
// include set var and send to the gripper
bool setReqPos(BYTE reqPos , int gripperIdx);
bool setReqSpeed(BYTE reqSpeed , int gripperIdx);
bool setReqForce(BYTE reqForce , int gripperIdx);


bool setGripperMode(char mode , int gripperIdx);

#endif