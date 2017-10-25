#ifndef __USER_H
#define __USER_H

// ���Ӷ���USE_����_BOARD
#define USE_LEFT_FRONT_BOARD
//RF ����
// ����ת������
#define yawBaseOff_RF 18146  //18126  //17540
#define hipBaseOff_RF 2346  //3706
#define kneeBaseOff_RF 15590  //900
// ����ת�����Ŷ��壬����
#define yawBaseSym_RF 1  //-1
#define hipBaseSym_RF 1
#define kneeBaseSym_RF -1
// ���ת����Ŷ��壬����
#define yawSpeedSym_RF 1  //-1
#define hipSpeedSym_RF -1
#define kneeSpeedSym_RF 1

//LF ����
#define yawBaseOff_LF 15724  //4398
#define hipBaseOff_LF 23034   //10994
#define kneeBaseOff_LF 24780  //33560  //23645 

#define yawBaseSym_LF 1  //-1
#define hipBaseSym_LF -1
#define kneeBaseSym_LF 1 

#define yawSpeedSym_LF -1
#define hipSpeedSym_LF -1
#define kneeSpeedSym_LF 1 
//RB ����
#define yawBaseOff_RB 15824  //17055  //17483   //17055
#define hipBaseOff_RB 19744  //30364  //30894  //8069
#define kneeBaseOff_RB 28050   //27860  //12335 

#define yawBaseSym_RB -1
#define hipBaseSym_RB 1
#define kneeBaseSym_RB -1

#define yawSpeedSym_RB 1
#define hipSpeedSym_RB 1
#define kneeSpeedSym_RB -1 
//LB ����
#define yawBaseOff_LB 19176  //19203  //18620 //+800
#define hipBaseOff_LB 5736  //5996  //5691	//+100
#define kneeBaseOff_LB 3740  //9030 //8770  //+465

#define yawBaseSym_LB -1
#define hipBaseSym_LB -1
#define kneeBaseSym_LB 1 

#define yawSpeedSym_LB -1
#define hipSpeedSym_LB 1
#define kneeSpeedSym_LB -1 

#define Joint_Num 3
	
#define Knee_Minangle_RF -17278+500   
#define Knee_Maxangle_RF -6317-500  
#define Hip_Maxangle_RF 9396-500
#define Hip_Minangle_RF -2628+500           
#define Yaw_Maxangle_RF 3000-500
#define Yaw_Minangle_RF -4164+500    

#define Knee_Minangle_LF -17278+500   
#define Knee_Maxangle_LF -5951-500   
#define Hip_Maxangle_LF 8907-500
#define Hip_Minangle_LF -2628+500    
#define Yaw_Maxangle_LF 4164-500
#define Yaw_Minangle_LF -3131+500  

#define Knee_Minangle_RB 5933+500   
#define Knee_Maxangle_RB 17819-500 
#define Hip_Maxangle_RB 2750-500   //2576-500
#define Hip_Minangle_RB -9047+500   //-8768+500          
#define Yaw_Maxangle_RB 3183-500   //4912-500	//3534
#define Yaw_Minangle_RB -4164+500  // -1980+500   //-3436

#define Knee_Minangle_LB 5602+500  //5916+500   
#define Knee_Maxangle_LB 17278-500  //16859-500  
#define Hip_Maxangle_LB 2541-500  //2567-500
#define Hip_Minangle_LB -9082+500  //-8602+500     
#define Yaw_Maxangle_LB 4146-500  //3148-500
#define Yaw_Minangle_LB -3532+500  //-4042+500    


#define speedMax 5000
#define speedMin -5000

#define PC_ID					0X00
#define POWER_BOARD_ID			0X01
#define LEFT_FRONT_BOARD_ID     0X02
#define LEFT_BACK_BOARD_ID      0X03
#define RIGHT_FRONT_BOARD_ID    0X04
#define RIGHT_BACK_BOARD_ID		0X05
#define HEADER_BOARD_ID      	0X06
#define TAIL_BOARD_ID			0X07

// �����л�����
#ifdef USE_RIGHT_BACK_BOARD
	#define FILTER_ID	RIGHT_BACK_BOARD_ID
	
	#define yawBaseOff yawBaseOff_RB
	#define hipBaseOff hipBaseOff_RB
	#define kneeBaseOff kneeBaseOff_RB
	
	#define yawBaseSym yawBaseSym_RB
	#define hipBaseSym hipBaseSym_RB
	#define kneeBaseSym kneeBaseSym_RB
	
	#define yawSpeedSym yawSpeedSym_RB
	#define hipSpeedSym hipSpeedSym_RB
	#define kneeSpeedSym kneeSpeedSym_RB	
	
	#define Knee_Minangle Knee_Minangle_RB
	#define Knee_Maxangle Knee_Maxangle_RB
	#define Hip_Maxangle Hip_Maxangle_RB
	#define Hip_Minangle Hip_Minangle_RB
	#define Yaw_Maxangle Yaw_Maxangle_RB
	#define Yaw_Minangle Yaw_Minangle_RB

#endif

#ifdef USE_LEFT_BACK_BOARD
	#define FILTER_ID	LEFT_BACK_BOARD_ID
	
	#define yawBaseOff yawBaseOff_LB
	#define hipBaseOff hipBaseOff_LB
	#define kneeBaseOff kneeBaseOff_LB
	
	#define yawBaseSym yawBaseSym_LB
	#define hipBaseSym hipBaseSym_LB
	#define kneeBaseSym kneeBaseSym_LB
	
	#define yawSpeedSym yawSpeedSym_LB
	#define hipSpeedSym hipSpeedSym_LB
	#define kneeSpeedSym kneeSpeedSym_LB
	
	#define Knee_Minangle Knee_Minangle_LB
	#define Knee_Maxangle Knee_Maxangle_LB
	#define Hip_Maxangle Hip_Maxangle_LB
	#define Hip_Minangle Hip_Minangle_LB
	#define Yaw_Maxangle Yaw_Maxangle_LB
	#define Yaw_Minangle Yaw_Minangle_LB	
	
#endif

#ifdef USE_LEFT_FRONT_BOARD
	#define FILTER_ID	LEFT_FRONT_BOARD_ID
	
	#define yawBaseOff yawBaseOff_LF
	#define hipBaseOff hipBaseOff_LF
	#define kneeBaseOff kneeBaseOff_LF
	
	#define yawBaseSym yawBaseSym_LF
	#define hipBaseSym hipBaseSym_LF
	#define kneeBaseSym kneeBaseSym_LF
	
	#define yawSpeedSym yawSpeedSym_LF
	#define hipSpeedSym hipSpeedSym_LF
	#define kneeSpeedSym kneeSpeedSym_LF
	
	#define Knee_Minangle Knee_Minangle_LF
	#define Knee_Maxangle Knee_Maxangle_LF
	#define Hip_Maxangle Hip_Maxangle_LF
	#define Hip_Minangle Hip_Minangle_LF
	#define Yaw_Maxangle Yaw_Maxangle_LF
	#define Yaw_Minangle Yaw_Minangle_LF	
	
#endif

#ifdef USE_RIGHT_FRONT_BOARD
	#define FILTER_ID	RIGHT_FRONT_BOARD_ID
	
	#define yawBaseOff yawBaseOff_RF
	#define hipBaseOff hipBaseOff_RF
	#define kneeBaseOff kneeBaseOff_RF
	
	#define yawBaseSym yawBaseSym_RF
	#define hipBaseSym hipBaseSym_RF
	#define kneeBaseSym kneeBaseSym_RF
	
	#define yawSpeedSym yawSpeedSym_RF
	#define hipSpeedSym hipSpeedSym_RF
	#define kneeSpeedSym kneeSpeedSym_RF
	
	#define Knee_Minangle Knee_Minangle_RF
	#define Knee_Maxangle Knee_Maxangle_RF
	#define Hip_Maxangle Hip_Maxangle_RF
	#define Hip_Minangle Hip_Minangle_RF
	#define Yaw_Maxangle Yaw_Maxangle_RF
	#define Yaw_Minangle Yaw_Minangle_RF
	
#endif

#endif



void UpdatePosition()
{
	kneeAngleoff = getJointCoder(kneeCoder);
	kneeAngle = 3600*kneeAngleoff/4096*10;    // 100���Ŵ󣬶���ֵ
	kneeAngle = kneeBaseSym*(kneeAngle - kneeBaseOff);  // ��ȥ��Ƚǻ�׼
	kneeAngle = kneeAngle*314.15/180;		 // ת���ȣ��Ŵ�100������10000��
	
	hipAngleoff  = getJointCoder(hipCoder); 
	hipAngle  = 36*hipAngleoff/4096*10;        //100���Ŵ�
	hipAngle = hipBaseSym*(hipAngle - hipBaseOff);     // ��ȥ��Ƚǻ�׼
	hipAngle = hipAngle*3.1415/180;
	
	yawAngleoff  = getJointCoder(yawCoder);
	yawAngle  =  36*yawAngleoff/4096*10;      //100���Ŵ�
	yawAngle = yawBaseSym*(yawAngle - yawBaseOff);    // ��ȥ��Ƚǻ�׼
	yawAngle = yawAngle*3.1415/180;
	
	pressValue = getJointCoder(pressSensor);			//��ȡ���ѹ��������ѹ��ֵ

}