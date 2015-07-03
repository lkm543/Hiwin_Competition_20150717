#include "LuoLitaArm.h"
#include "finger.h"
#include "gripper_2.h"
using namespace std;
using namespace Eigen;
/*
LKM 
20150625 Modified on HCW v4
*/

//-----------------------------
//------Global Variable--------
//-----------------------------
//IMPCard.cpp------------------

//-----------------------------
//RobotLita.cpp----------------

//-----------------------------
//ControlLita.cpp--------------

//-----------------------------
//LuoLitaArmFunc.cpp-----------

//-----------------------------
	Eigen::Matrix4f init_pose = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f first_corner = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f second_corner = Eigen::Matrix4f::Identity();
	Eigen::Matrix2f desk_transform = Eigen::Matrix2f::Identity();
	Eigen::Matrix4f first_block = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f first_block_temp = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f first_target = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f first_target_temp = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f next_block = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f next_block_temp = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f next_target = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f next_target_temp = Eigen::Matrix4f::Identity();
	bool block=false; // In moving mode or not
	bool display_mode=true; //unknown

// function prototype for periodic timer function
	void RTFCNDCL TimerHandler1( void * nContext ); //unknown
	void setDefaultArmSpeed(float percentage = 1.0f);

// main 
int main( int argc, char **argv, char **envp)
{
	//Variable
	int Value_Gripper_Open=150; //0-255
	int Value_Postion_Gripper_Close=255; //0-255
	int Value_Postion_Gripper_Force=80; //0-255
	int Value_Gripper_Speed=200; //0-255
	int Value_Arm_Speed=1.0; //0-1.2 , float
	int Value_Arm_Speed_Aim=0.3; //0-1.2 , float ,moving on 'j' mode
	int Value_Height_While_Moving=0.8;
	double x_offset=0;
	double y_offset=0;
	double z_offset=0;

	//For Gripper
	init_gripper_para(); //unknown
	initialize_gripper_motion(); //unknown
	GoToPosReqFlag = true; //unknown
	setGoToPosReqFlag( GoToPosReqFlag ); //unknown
	setReqSpeed(Value_Gripper_Speed); //gripper speed
	setReqForce(Value_Postion_Gripper_Force); //gripper force

	init_LuoLita_1(); //move to initial position 1

    // for periodic timer code
    LARGE_INTEGER  liPeriod_1ms;   // timer period ,unknown
    HANDLE         hTimer1;     // timer handle ,unknown

    //  RTX periodic timer code: unknown
    //  TO DO: Set default timer period to your desired time.
    //         The period needs to be an even multiple of the HAL
    //         period found in the control panel.
    //         This example uses a period of 500 micro seconds.

    liPeriod_1ms.QuadPart  = 10000;
	//liPeriod_10ms.QuadPart = 100000;

	Init_IMPCard();

    // Create a periodic timer
    if (! (hTimer1 = RtCreateTimer(
                                   NULL,            // security
                                   0,               // stack size - 0 uses default
                                   TimerHandler1,    // timer handler
                                   NULL,            // NULL context (argument to handler)
                                   RT_PRIORITY_MAX, // priority
                                   CLOCK_2) ))      // RTX HAL timer
    {
        //
        // TO DO:  exception code here
        // RtWprintf(L"RtCreateTimer error = %d\n",GetLastError());
        ExitProcess(1);
    }

    if (! RtSetTimerRelative( hTimer1, &liPeriod_1ms, &liPeriod_1ms ))
    {
        //RtWprintf(L"RtSetTimerRelative error = %d\n",GetLastError());
		// TO DO: exception code here
        ExitProcess(1);
    }
	init_LuoLita_2(); //unknown

	init_pose.block(0,0,3,3) = R07Cmd;//save initial rotation
	init_pose.block(0,3,3,1) = P07Cmd;//save initial translation
	setDefaultArmSpeed(1.0);
	
	kbCmd='m';
	keyboard_cw();
	kbCmd='j';
	keyboard_cw();

	/////////////////////////////////////////
while(1)
	{
			char Cmd=' ';

			if (display_mode==false)
			{
					if ( _kbhit() )
						 Cmd = _getche();
			}


		    system("cls");
            if (display_mode==true)
			{
					if (ModeArm==2) 
					{
						printf("Cartesian Mode    ");
						if (SubMode==2) printf("translation\n"); 
						else if (SubMode==4) printf("rotation\n"); 
						else if (SubMode==0) printf("LOCK\n");
					}
					else if (ModeArm==1) printf("Joint Mode    ");
			
					printf("Input commend and press enter:\n");
					printf("[1] Set desk calibration Position\n");
					printf("[2] Run desk calibration\n");
					printf("[2] Teach and save it into txt\n");
					printf("[3] Load one path by txt\n");
					printf("[4] Load multiple path by txt\n");
                    printf("[5] Relative position\n");
					printf("[6] Move mode\n");
					printf("[z] Go to initial pose\n");
					printf("[,] open gripper\n");
					printf("[.] close gripper\n");
					printf("[p] Change display mode\n");
					printf("[s] Setting mode\n");
					printf("[q] Quit\n");
					printf(">>");
					std::cin>>Cmd;
			}

		if ( Cmd == 'q' )
		{
			break;
		}
		switch (Cmd)
		{
			case '1':
			{
				printf("Set Calibration Point\n");
				printf("Move the Gripper to The Edge of Table\n");
				string file="calibration.txt";
				printf("Press [enter] to Save Calibration 1 \n");
				block=true;
				bool first=false;
				bool second=false;
				while (block)
				{
					if ( _kbhit() )
                    {
                        kbCmd = _getche();
                        switch(kbCmd)
                        {
							case ',':
							{
								setReqPos(Value_Gripper_Open);
								break;
							}
							case '.':
							{
								setReqPos( Value_Postion_Gripper_Close);
								break;
							}
							case 13://Enter
							{
								if (first==false)
								{
									kbCmd = 'j';
									keyboard_cw();
									first_block.block(0,0,3,3)=R07Cmd;
									first_block.block(0,3,3,1)=P07Cmd;
									printf("Press [enter] to Save Calibration 2\n");
									first=true;
								}
								else if (second==false)
								{
									kbCmd = 'j';
									keyboard_cw();
									first_target.block(0,0,3,3)=R07Cmd;
									first_target.block(0,3,3,1)=P07Cmd;
									printf("Finish Setting, new Saving\n");
									second=true;
									block=false;
								}
	
								break;
							}
							default:
							{
								char kb_tmp = kbCmd;
								kbCmd = 'j';
								keyboard_cw();
								kbCmd = kb_tmp;
								setDefaultArmSpeed(Value_Arm_Speed_Aim);
								keyboard_cw();
								setDefaultArmSpeed(Value_Arm_Speed);
							}
						}
					}
				}
				ofstream fout(file); //(file,ios::app) 
				    if(!fout) { 
				        cout << "無法寫入檔案\\n"; 
				    }
				fout << first_block << endl;
				fout << first_target << endl;
				fout.close(); 
			break;
				/*
				block=true;
				while (block)
				{
					if ( _kbhit() )
                    {
                        kbCmd = _getche();
						
                        switch(kbCmd)
                        {
							case ',':
							{
								//finger.move(80);
								setReqPos(80);
								break;
							}
							case '.':
							{
								//finger.move(120);
								setReqPos(255);
								break;
							}
							case 13://Enter
							{
								kbCmd = 'j';
								keyboard_cw();
								block=false;
								break;
							}
							
							default:
							{
								char kb_tmp = kbCmd;
								kbCmd = 'j';
								keyboard_cw();
								kbCmd = kb_tmp;
								keyboard_cw();
							}
						}
					} 
				} // while end
			break;*/
			}
			case '2':
			{
				printf("Desk Calibration Start\n");
				printf("Press [Enter] to Start Calibration\n");
				system("pause");

				//ifstream fin0("offset.txt");
				//fin0>>x_offset >> y_offset >> z_offset;
				//cout<<"x offset:"<<x_offset<<"  y offset:" << y_offset <<"   z offset:" << z_offset<<endl;

				ifstream fin2("calibration.txt");
				if(!fin2) { 
					cout << "無法讀入檔案\\n"; 
				} 
				for (int i=0;i<=3;i++)
				{
					fin2 >> first_block(i,0) >> first_block(i,1) >> first_block(i,2) >>first_block(i,3);
				}
				first_block(0,3)=first_block(0,3)+x_offset;
				first_block(1,3)=first_block(1,3)+y_offset;
				first_block(2,3)=first_block(2,3)+z_offset;
				//cout <<first_block<<endl; 
				for (int i=0;i<=3;i++)
				{
					fin2 >> first_target(i,0) >> first_target(i,1) >> first_target(i,2) >>first_target(i,3);
				}
				first_target(0,3)=first_target(0,3)+x_offset;
				first_target(1,3)=first_target(1,3)+y_offset;
				first_target(2,3)=first_target(2,3)+z_offset;
				
				setReqPos(Value_Postion_Gripper_Close);
				Sleep(2000);

				Move_L_Abs( first_block ,0.0f);
				while(MOVL) {}
				Sleep(1000);

				Move_L_Abs( first_target ,0.0f);
				while(MOVL) {}
				Sleep(1000);
				/*
				block=true;
				bool first=false;
				bool second=false;
				while (block)
				{
					if ( _kbhit() )
                    {
                        kbCmd = _getche();
                        switch(kbCmd)
                        {

							case 13://Enter
							{
								if (first==false)
								{
									kbCmd = 'j';
									keyboard_cw();
									first_corner.block(0,0,3,3)=R07Cmd;
									first_corner.block(0,3,3,1)=P07Cmd;
									printf("press [enter] to save second corner\n");
									first=true;
								}
								else if (second==false)
								{
									kbCmd = 'j';
									keyboard_cw();
									second_corner.block(0,0,3,3)=R07Cmd;
									second_corner.block(0,3,3,1)=P07Cmd;
									second=true;
									block=false;
								}
	
								break;
							}
							default:
							{
								char kb_tmp = kbCmd;
								kbCmd = 'j';
								keyboard_cw();
								kbCmd = kb_tmp;
								keyboard_cw();
							}
						}
					}
				}
				
				cout<<first_corner<<endl;
				cout<<"---------------"<<endl;
				cout<<second_corner<<endl;
				//if
				double theta=-atan2((second_corner(0,3)-first_corner(0,3)),(second_corner(1,3)-first_corner(1,3)));
				desk_transform(0,0)=cos(theta);
				desk_transform(0,1)=-sin(theta);
				desk_transform(1,0)=sin(theta);
				desk_transform(1,1)=cos(theta);
				cout<<desk_transform<<endl;
				cout<<theta<<endl;
				ofstream fout("calibration.txt"); 
				    if(!fout) { 
				        cout << "無法寫入檔案\\n"; 
				    }
				fout << desk_transform<< endl;
				fout.close(); 
				//Sleep(3000);
				*/
			break;
			
			}
			case '3':  //teach
			{
				printf("teach mode\n");
				char file[10];
				printf("save to :\n");
				std::cin>>file;
				sprintf(file, "%s.txt", file);
				printf("press [enter] to save 1\n");
				block=true;
				bool first=false;
				bool second=false;
				while (block)
				{
					if ( _kbhit() )
                    {
                        kbCmd = _getche();
                        switch(kbCmd)
                        {
							case ',':
							{
								//finger.move(80);
								setReqPos(80);
								break;
							}
							case '.':
							{
								//finger.move(120);
								setReqPos(255);
								break;
							}
							case 13://Enter
							{
								if (first==false)
								{
									kbCmd = 'j';
									keyboard_cw();
									first_block.block(0,0,3,3)=R07Cmd;
									first_block.block(0,3,3,1)=P07Cmd;
									printf("press [enter] to save 2\n");
									first=true;
								}
								else if (second==false)
								{
									kbCmd = 'j';
									keyboard_cw();
									first_target.block(0,0,3,3)=R07Cmd;
									first_target.block(0,3,3,1)=P07Cmd;
									second=true;
									block=false;
								}
	
								break;
							}
							default:
							{
								char kb_tmp = kbCmd;
								kbCmd = 'j';
								keyboard_cw();
								kbCmd = kb_tmp;
								setDefaultArmSpeed(0.3);
								keyboard_cw();
								setDefaultArmSpeed(1.0);
							}
						}
					}
				}
				ofstream fout(file); //(file,ios::app) 
				    if(!fout) { 
				        cout << "無法寫入檔案\\n"; 
				    }
				fout << first_block << endl;
				fout << first_target << endl;
				fout.close(); 
			break;
			}
			case '4': //read
			{
				
				//ifstream fin1("calibration.txt"); 
				//if(!fin1) { 
				//	cout << "無法讀入檔案\\n"; 
				//} 
				//for (int i=0;i<=1;i++)
				//{
				//	fin1 >> desk_transform(i,0) >> desk_transform(i,1);
				//}
				//cout<<desk_transform<<endl;
				ifstream fin0("offset.txt");
				double x_offset=0,y_offset=0,z_offset=0 ;
				fin0>>x_offset >> y_offset >> z_offset;
				cout<<"x offset:"<<x_offset<<"  y offset:" << y_offset <<"   z offset:" << z_offset<<endl;
				char file[10];
				printf("read file : \n");
				std::cin>>file;
				sprintf(file, "%s.txt", file);
				ifstream fin2(file);
				if(!fin2) { 
					cout << "無法讀入檔案\\n"; 
				} 
				for (int i=0;i<=3;i++)
				{
					fin2 >> first_block(i,0) >> first_block(i,1) >> first_block(i,2) >>first_block(i,3);
				}
				first_block(0,3)=first_block(0,3)+x_offset;
				first_block(1,3)=first_block(1,3)+y_offset;
				first_block(2,3)=first_block(2,3)+z_offset;
				cout <<first_block<<endl; 
				for (int i=0;i<=3;i++)
				{
					fin2 >> first_target(i,0) >> first_target(i,1) >> first_target(i,2) >>first_target(i,3);
				}
				first_target(0,3)=first_target(0,3)+x_offset;
				first_target(1,3)=first_target(1,3)+y_offset;
				first_target(2,3)=first_target(2,3)+z_offset;
				cout <<first_target<<endl;
				Sleep(2000);
			break;
			}
			
			case '5'://play
			{
				int loop = 216;
				
				//ifstream fin0("offset.txt");
				//double x_offset=0,y_offset=0,z_offset=0 ;
				//fin0>>x_offset >> y_offset >> z_offset;
				//cout<<"x offset:"<<x_offset<<"  y offset:" << y_offset <<"   z offset:" << z_offset<<endl;
				
				for (int i=201;i<loop;i++)
				{
					//read file
					char file[10];
					printf("read file : \n");
					sprintf(file, "%d.txt", i);
					ifstream fin2(file);
					if(!fin2) { 
						cout << "無法讀入檔案"<<file<<"\\n"; 
					} 
					for (int i=0;i<=1;i++)
					{
						fin2 >> first_block(i,3);
					}
					first_block(0,0)=0.06;
					first_block(0,1)=-1.0;
					first_block(0,2)=0.0;
					first_block(1,0)=-1.0;
					first_block(1,1)=-0.06;
					first_block(1,2)=0.0074;
					first_block(2,0)=-0.0074;
					first_block(2,1)=0.0;
					first_block(2,2)=-1.0;
					first_block(2,3)=-0.15;
					first_block(3,0)=0.0;
					first_block(3,1)=0.0;
					first_block(3,2)=0.0;
					first_block(3,3)=1.0;
					//>> first_block(i,1) >> first_block(i,2) >>
					//first_block(0,3)=first_block(0,3)+x_offset;
					//first_block(1,3)=first_block(1,3)+y_offset;
					//first_block(2,3)=first_block(2,3)+z_offset;
					cout <<first_block<<endl; 
					for (int i=0;i<=3;i++)
					{
						fin2 >> first_target(i,0) >> first_target(i,1) >> first_target(i,2) >>first_target(i,3);
					}
					
					first_target(2,3)=-0.15;
					//first_target(0,3)=first_target(0,3)+x_offset;
					//first_target(1,3)=first_target(1,3)+y_offset;
					//first_target(2,3)=first_target(2,3)+z_offset;
					//cout <<fin2<<endl;
					cout <<first_target<<endl;
					Sleep(2000);

					kbCmd = 'j';
				keyboard_cw();
				float pregrab_h=0.08;

				first_block_temp=first_block;
				first_target_temp=first_target;
				setReqPos(150);
				Sleep(1000);
				if (P07Cmd(2,0)>first_block_temp(2,3)+pregrab_h) //如果現在高度>目標高度，則保持高度移動到目標點
				{
					first_block_temp(2,3)=P07Cmd(2,0);
				}
				else //否則，移動到目標上方稍微高一點的位置
				{
					first_block_temp(2,3)=first_block_temp(2,3)+pregrab_h;
				}
				//cout<<"first_block_temp: "<<first_block_temp(0,3)<<",  "<<first_block_temp(1,3)<<",  "<<first_block_temp(2,3)<<endl;
				//system("pause");
				Move_L_Abs( first_block_temp ,0.0f);
				while(MOVL) {}
				first_block_temp(2,3)=first_block(2,3);//下去
				Move_L_Abs( first_block_temp ,0.0f);
				while(MOVL) {}
				//finger.move(120);//夾
				setReqPos(255);
				Sleep(1000);
				first_block_temp(2,3)=first_target_temp(2,3)+pregrab_h; //抬高到target高度再高一點
				Move_L_Abs( first_block_temp ,0.0f);
				while(MOVL) {}

				first_target_temp(2,3)=first_target_temp(2,3)+pregrab_h; //移動到target位置
				Move_L_Abs( first_target_temp ,0.0f);
				while(MOVL) {}
				first_target_temp(2,3)=first_target(2,3); //往下
				Move_L_Abs( first_target_temp ,0.0f);
				while(MOVL) {}
				//finger.move(80);//放開
				setReqPos(150);
				Sleep(1000);
				first_target_temp(2,3)=first_target_temp(2,3)+pregrab_h; //稍微往上
				Move_L_Abs( first_target_temp ,0.0f);
				while(MOVL) {}

				}

				ifstream fin2("216.txt");
				if(!fin2) { 
					cout << "無法讀入檔案\\n"; 
				} 
				for (int i=0;i<=3;i++)
				{
					fin2 >> first_block(i,0) >> first_block(i,1) >> first_block(i,2) >>first_block(i,3);
				}
				//first_block(0,3)=first_block(0,3)+x_offset;
				//first_block(1,3)=first_block(1,3)+y_offset;
				//first_block(2,3)=first_block(2,3)+z_offset;
				cout <<first_block<<endl; 
				for (int i=0;i<=3;i++)
				{
					fin2 >> first_target(i,0) >> first_target(i,1) >> first_target(i,2) >>first_target(i,3);
				}
				//first_target(0,3)=first_target(0,3)+x_offset;
				//first_target(1,3)=first_target(1,3)+y_offset;
				//first_target(2,3)=first_target(2,3)+z_offset;

				kbCmd = 'j';
				keyboard_cw();
				//float pregrab_h=0.1;
				int pregrab_h=0.08;
				first_block_temp=first_block;
				first_target_temp=first_target;
				setReqPos(255);
				Sleep(1000);
				if (P07Cmd(2,0)>first_block_temp(2,3)+pregrab_h) //如果現在高度>目標高度，則保持高度移動到目標點
				{
					first_block_temp(2,3)=P07Cmd(2,0);
				}
				else //否則，移動到目標上方稍微高一點的位置
				{
					first_block_temp(2,3)=first_block_temp(2,3)+pregrab_h;
				}
				Move_L_Abs( first_block_temp ,0.0f);
				while(MOVL) {}
				first_block_temp(2,3)=first_block(2,3);//下去
				Move_L_Abs( first_block_temp ,0.0f);
				while(MOVL) {}

				first_target_temp(2,3)=first_target_temp(2,3); 
				Move_L_Abs( first_target_temp ,0.0f);
				while(MOVL) {}
			break;
			}
			case '6':
			{
				kbCmd = 'j';
				keyboard_cw();
				float pregrab_h=0.1;

				first_block_temp=first_block;
				first_target_temp=first_target;
				setReqPos(150);
				Sleep(1000);
				if (P07Cmd(2,0)>first_block_temp(2,3)+pregrab_h) //如果現在高度>目標高度，則保持高度移動到目標點
				{
					first_block_temp(2,3)=P07Cmd(2,0);
				}
				else //否則，移動到目標上方稍微高一點的位置
				{
					first_block_temp(2,3)=first_block_temp(2,3)+pregrab_h;
				}
				Move_L_Abs( first_block_temp ,0.0f);
				while(MOVL) {}
				first_block_temp(2,3)=first_block(2,3);//下去
				Move_L_Abs( first_block_temp ,0.0f);
				while(MOVL) {}
				//finger.move(120);//夾
				setReqPos(255);
				Sleep(1000);
				first_block_temp(2,3)=first_target_temp(2,3)+pregrab_h; //抬高到target高度再高一點
				Move_L_Abs( first_block_temp ,0.0f);
				while(MOVL) {}

				first_target_temp(2,3)=first_target_temp(2,3)+pregrab_h; //移動到target位置
				Move_L_Abs( first_target_temp ,0.0f);
				while(MOVL) {}
				first_target_temp(2,3)=first_target(2,3); //往下
				Move_L_Abs( first_target_temp ,0.0f);
				while(MOVL) {}
				//finger.move(80);//放開
				setReqPos(150);
				Sleep(1000);
				first_target_temp(2,3)=first_target_temp(2,3)+pregrab_h; //稍微往上
				Move_L_Abs( first_target_temp ,0.0f);
				while(MOVL) {}

			break;
			}
			case '7':   //相對位置
			{
				kbCmd = 'j';
				keyboard_cw();
				char file[10];
				
				ifstream fin1("0.txt");
				if(!fin1) { 
					cout << "無法讀入檔案\\n"; 
					} 
				
				for (int i=0;i<=3;i++)
				{
					fin1 >> first_block(i,0) >> first_block(i,1) >> first_block(i,2) >>first_block(i,3);
				}
				cout<<"第一個積木位置:"<<endl;
				cout <<first_block<<endl; 
				for (int i=0;i<=3;i++)
				{
					fin1 >> first_target(i,0) >> first_target(i,1) >> first_target(i,2) >>first_target(i,3);
				}
				cout <<"第一個積木擺放:"<<endl;
				cout <<first_target<<endl;

				cout<<"輸入要抓取第幾個積木:";
					std::cin>>file;
					sprintf(file, "%s.txt",file );
					ifstream fin2(file);
					if(!fin2) { 
						cout << "無法讀入檔案\\n"; 
					} 
					
					for (int i=0;i<=3;i++)
					{
						fin2 >> next_block(i,0) >> next_block(i,1) >> next_block(i,2) >> next_block(i,3);
					}
					cout <<next_block<<endl; 
					for (int i=0;i<=3;i++)
					{
						fin2 >> next_target(i,0) >> next_target(i,1) >> next_target(i,2) >> next_target(i,3);
					}
					cout <<next_target<<endl;
					
					//計算相對位置到絕對位置
					next_block.block(0,0,3,3)=next_block.block(0,0,3,3)*first_block.block(0,0,3,3);
					next_block.block(0,3,3,1)=next_block.block(0,3,3,1)+first_block.block(0,3,3,1);
					next_target.block(0,0,3,3)=next_target.block(0,0,3,3)*first_target.block(0,0,3,3);
					next_target.block(0,3,3,1)=next_target.block(0,3,3,1)+first_target.block(0,3,3,1);
					cout<<"計算後"<<endl;
					cout<<next_block<<endl;
					cout<<next_target<<endl;
					Sleep(1000);
					//
					float pregrab_h=0.05;

					next_block_temp=next_block;
					next_target_temp=next_target;
				
					next_block_temp(2,3)=next_block_temp(2,3)+pregrab_h;//稍微高一點
					Move_L_Abs( next_block_temp ,0.0f);
					while(MOVL) {}
					next_block_temp(2,3)=next_block_temp(2,3)-pregrab_h;//下去
					Move_L_Abs( next_block_temp ,0.0f);
					while(MOVL) {}
					//finger.move(120);//夾
					setReqPos(255);
					Sleep(1000);
					next_block_temp(2,3)=next_target_temp(2,3)+pregrab_h; //抬高到target高度再高一點
					Move_L_Abs( next_block_temp ,0.0f);
					while(MOVL) {}

					next_target_temp(2,3)=next_target_temp(2,3)+pregrab_h; //移動到target位置
					Move_L_Abs( next_target_temp ,0.0f);
					while(MOVL) {}
					next_target_temp(2,3)=next_target_temp(2,3)-pregrab_h; //往下
					Move_L_Abs( next_target_temp ,0.0f);
					while(MOVL) {}
					//finger.move(80);//放開
					setReqPos(220);
					Sleep(1000);
					next_target_temp(2,3)=next_target_temp(2,3)+pregrab_h; //稍微往上
					Move_L_Abs( next_target_temp ,0.0f);
					while(MOVL) {}
				

			break;
			}
			case '8':   //
			{
				kbCmd = 'j';
				keyboard_cw();
				char file[10];
				cout<<"輸入";
				std::cin>>file;
				sprintf(file, "%s.txt",file );
				ifstream fin2(file);
				if(!fin2) { 
						cout << "無法讀入檔案\\n"; 
					} 
					
				for (int i=0;i<=3;i++)
					{
						fin2 >> next_block(i,0) >> next_block(i,1) >> next_block(i,2) >> next_block(i,3);
					}
					cout <<next_block<<endl; 
				Move_L_Abs( next_block ,0.0f);
				while(MOVL) {}
				break;
			}
			case 't':
			{
				kbCmd = 'j';
				keyboard_cw();
				Move_L_Abs( first_corner ,0.0f);
				while(MOVL) {}
				Eigen::Matrix4f temp = Eigen::Matrix4f::Identity();
				temp.block(0,0,3,3)=R07Cmd;
				temp.block(0,3,3,1)=P07Cmd;   //
				//temp(1,3)=temp(1,3)+0.1;
				//cout<<temp<<endl;
				//temp(0,3)=(temp(0,3)-first_corner(0,3))*desk_transform(0,0)+(temp(1,3)-first_corner(1,3))*desk_transform(0,1)+first_corner(0,3);
				//temp(1,3)=(temp(0,3)-first_corner(0,3))*desk_transform(1,0)+(temp(1,3)-first_corner(1,3))*desk_transform(1,1)+first_corner(1,3);
				temp(0,3)=0*desk_transform(0,0)+0.1*desk_transform(0,1)+first_corner(0,3);
				temp(1,3)=0*desk_transform(1,0)+0.1*desk_transform(1,1)+first_corner(1,3);
				cout<<temp<<endl;
				//Move_L_Abs( temp ,0.0f);
				//while(MOVL) {}
				Sleep(3000);
				break;
			}
			

			//case 'y':
			//{
			//	kbCmd = 'j';
			//	keyboard_cw();
			//	Move_L_Abs( first_corner ,0.0f);
			//	while(MOVL) {}
			//	Eigen::Vector3f temp_1=P07Cmd;
			//	Eigen::Matrix4f temp = Eigen::Matrix4f::Identity();
			//	temp.block(0,0,3,3)=R07Cmd;
			//	temp.block(0,3,3,1)=P07Cmd;   //
			//	break;
			//}
			case 'z': //回到init位置
				kbCmd = 'j';
				keyboard_cw();
				Move_L_Abs( init_pose ,0.0f);
				while(MOVL) {} //wait for motion to complete
				break;
			//case 'x':  //紀錄角落
			//	{
			//	ofstream fout("temp2.txt"); 
			//	    if(!fout) { 
			//	        cout << "無法寫入檔案\\n"; 
			//	        return 1; 
			//	    }
			//	corner_pose.block(0,0,3,3) = R07Cmd;//save rotation
			//	corner_pose.block(0,3,3,1) = P07Cmd;//save translation
			//	fout << corner_pose << endl; 
			//	fout.close(); 
			//	}
			//	break;
			//case 'a':  //x + 
			//	next_pose << 1, 0, 0, 0.03,
			//				 0, 1, 0, 0,
			//				 0, 0, 1, 0,
			//				 0, 0, 0, 1;
			//	next_pose=next_pose*corner_pose;
			//	cout<<next_pose;
			//	Move_L_Abs( next_pose ,0.0f);
			//	while(MOVL) {} //wait for motion to complete
			//	break;
			//case 'b':  //y +
			//	next_pose << 1, 0, 0, 0,
			//				 0, 1, 0, 0.03,
			//				 0, 0, 1, 0,
			//				 0, 0, 0, 1;
			//	next_pose=next_pose*corner_pose;
			//	cout<<next_pose;
			//	Move_L_Abs( next_pose ,0.0f);
			//	while(MOVL) {} //wait for motion to complete
			//	break;

			//case 'v': //cartesian mode下改變z軸 -30度
			//	{
			//	Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();
			//	rotation<< 0.866, -0.5,   0,
			//			   0.5,  0.866, 0,
			//			      0,    0,   1;
			//	next_pose.block(0,0,3,3) = rotation*R07Cmd;
			//	next_pose.block(0,3,3,1) = P07Cmd;
			//	Move_L_Abs( next_pose ,0.0f);
			//	while(MOVL) {} //wait for motion to complete
			//	}
			//	break;
			//case 'c': //cartesian mode下改變z軸 30度
			//	{
			//	Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();
			//	rotation<< 0.866, 0.5,   0,
			//			   -0.5,  0.866, 0,
			//			      0,    0,   1;
			//	next_pose.block(0,0,3,3) = rotation*R07Cmd;
			//	next_pose.block(0,3,3,1) = P07Cmd;
			//	Move_L_Abs( next_pose ,0.0f);
			//	while(MOVL) {} //wait for motion to complete
			//	}
			//	break;
			case ',': //夾爪open
				//finger.move(80);
				setReqPos(80);
				break;
			case '.': //夾爪close
				//finger.move(120);
				setReqPos(255);
				break;
			case 'p':  
				display_mode=!display_mode;
				break;
		}
		Cmd = ' ';

		//MainLoop_keyboard();
		
		Sleep(29);
		//system("cls");
		if (display_mode==false)
		DisplayLoop(); //display function by laoda
	}

	

	ByeBye();
	while(1)
	{
		if ( _kbhit() )
		{
			 kbCmd = _getch();
			 cout << kbCmd << endl;
			 if ( kbCmd == kb_ESC )
			 {
			     break;
		 	 }
		}
		Sleep(30);
		system("cls");
		DisplayLoop();
	}

	Sleep(100);
	if(!RtDeleteTimer( hTimer1 ) )
	{
        //RtWprintf(L"RtDeleteTimer error = %d\n",GetLastError());
		// TO DO:  your exception code here
		Close_IMPCard();
        ExitProcess(1);
	}
	
	Close_IMPCard();
	Sleep(1000);

	OutputData();

	ExitProcess(0);
}
// main end

//
// RTX periodic timer handler function
// Refer to the RTX Samples directory for examples
// of periodic timers
//
void RTFCNDCL TimerHandler1( PVOID context )
{
	ServoLoop();
}

void setDefaultArmSpeed(float percentage)
{
    Ang_Vel_limit = ( 0.01f*100 * deg2rad*RVmax / 1000.0f ) *SAMPLING_TIME_C * percentage;
    Ang_Acc_limit = ((0.01f*100 * deg2rad*RAmax / 1000.0f ) *SAMPLING_TIME_C) /1000 *SAMPLING_TIME_C * percentage;
    Ang_Dec_limit = Ang_Acc_limit * percentage;
    Lin_Vel_limit = ( 0.01f*160 * 0.001f*LVmax / 1000.0f ) *SAMPLING_TIME_C * percentage;
    Lin_Acc_limit = ((0.01f*320 * 0.001f*LAmax / 1000.0f ) *SAMPLING_TIME_C) /1000 *SAMPLING_TIME_C * percentage;
    Lin_Dec_limit = Lin_Acc_limit * percentage;
    Jn_Vel_limit = ( 0.01f*25* deg2rad*RVmax / 1000.0f ) *SAMPLING_TIME_C * percentage;
    Jn_Acc_limit = ((0.01f*25* deg2rad*RAmax / 1000.0f ) *SAMPLING_TIME_C) /1000 *SAMPLING_TIME_C * percentage;
    Jn_Dec_limit = Jn_Acc_limit * percentage;
}