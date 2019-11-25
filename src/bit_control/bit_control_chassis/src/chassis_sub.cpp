/*
 * @Descripttion:  4轮车底盘驱动程序
 * @version: 1.0
 * @Author: ifan Ma
 * @Date: 2019-09-18 21:30:31
 * @LastEditors: Ifan Ma
 * @LastEditTime: 2019-09-21 13:42:01
 */
 
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>


#   include <stdio.h>
#   include <stdlib.h>
#   include <string.h>
#   include <strings.h>
#   include <unistd.h>
#   include <sys/types.h>
#   include <sys/stat.h>
#   include <sys/time.h>
#   include <fcntl.h>
#   include <pthread.h>
#   include <math.h>
#   include <sys/ioctl.h> 
#   include <queue>
#   include "controlcan.h"
#   define msleep(ms)  usleep((ms)*1000)
#   define min(a,b)  (((a) < (b)) ? (a) : (b))


#define MAX_CHANNELS  4     //Can设备的通道最大值，现在其实USBCan2只有两个 
#define RX_WAIT_TIME  30   //接收数据的TimeOut
#define RX_BUFF_SIZE  3  //接收缓存区大小

//======定义配置Can设备的一些数据======//

unsigned gDevType = 0;
unsigned gDevIdx = 0;
unsigned gChMask = 0;
unsigned gBaud = 0;
unsigned gTxType = 0;

//=====用到的结构体实例化=============//

VCI_INIT_CONFIG config;
VCI_CAN_OBJ can;
struct timeval tm1,tm2;
	
RX_CTX rx_ctx[MAX_CHANNELS];
pthread_t rx_threads[MAX_CHANNELS];
Motor motor[8];             //电机结构，内有电机的指令，电机的状态还没有回读

//======配置电机需要的变量及命令，在初始化中赋给结构体====//

int FindID[8]    = { 1 , 2 , 3 , 4 , 5 , 6 , 7 , 8 };
int dftCan[8]    = { CAN1 , CAN1 , CAN1 , CAN1 , CAN1 , CAN1 , CAN1 , CAN1 };
//BYTE dftMode[8]  = { M_pos, M_pos, M_pos, M_pos, M_spd, M_spd, M_spd, M_spd};
BYTE Stop[8]     = { 0x00 , 0xDA , 0x00 , 0x10 , 0x00 , 0x00 , 0x00 , 0x0F };
BYTE Enable[8]   = { 0x00 , 0xDA , 0x00 , 0x10 , 0x00 , 0x00 , 0x00 , 0x1F };
BYTE PosMode[8]  = { 0x00 , 0xDA , 0x00 , 0x19 , 0x00 , 0x00 , 0x00 , 0x3F };
BYTE PosAbslt[8] = { 0x00 , 0xDA , 0x00 , 0x17 , 0x00 , 0x00 , 0x00 , 0x4F };
BYTE PosRltvy[8] = { 0x00 , 0xDA , 0x00 , 0x17 , 0x00 , 0x00 , 0x00 , 0x5F };
BYTE SetPos[8]   = { 0x00 , 0xDA , 0x00 , 0x16 , 0x00 , 0x00 , 0x00 , 0x00 };   //后4Bytes为位置，32位
BYTE SpdMode[8]  = { 0x00 , 0xDA , 0x00 , 0x19 , 0x00 , 0x00 , 0x00 , 0x2F };
BYTE SetSpd[8]   = { 0x00 , 0xDA , 0x00 , 0x11 , 0x00 , 0x00 , 0x05 , 0x55 };   //后4Bytes为速度，32位
BYTE EmgStop[8]  = { 0x00 , 0xDA , 0x00 , 0x30 , 0x00 , 0x00 , 0x00 , 0x1F };
BYTE ClrAlarm[8] = { 0x00 , 0xDA , 0x00 , 0x15 , 0x00 , 0x00 , 0x00 , 0x7F };
BYTE AskAlarm[8] = { 0x00 , 0xDC , 0x00 , 0xE3 , 0x00 , 0x00 , 0x00 , 0x80 };
BYTE SetPosSpd[8]= { 0x00 , 0xDA , 0x00 , 0x14 , 0x00 , 0x00 , 0x01 , 0x11 };
BYTE SetUpDnTime[8] = { 0x00 , 0xDA , 0x00 , 0x12 , 0x00 , 0x00 , 0x0A , 0x0A };
BYTE AskPos[8]   = { 0x00 , 0xDC , 0x00 , 0xE8 , 0x00 , 0x00 , 0x00 , 0x00 };
BYTE WriteI[8]   = { 0x00 , 0xDA , 0x00 , 0x2D , 0x00 , 0x00 , 0x1F , 0x40 };
int RecRight = 0;    //接受回传响应状态位
bool NeedRecData = 0;
BYTE MotorOnControl = 0;
int n = 0;

int8_t m3dir = 1;
int8_t m2dir = 1;
int8_t m1dir = 1;
int8_t m0dir = 1;
double m3p_last = 0;
double m2p_last = 0;
double m1p_last = 0;
double m0p_last = 0;
double m3p = 0;
double m2p = 0;
double m1p = 0;
double m0p = 0;


//====================车辆数据=======================//

#define A 36.5      //36.5    //车宽一半 Cm
#define B 44.64    //车长一半 Cm
#define LineNum 10000.0f   //转向编码器线数
#define DegreeLmt 20    //度
#define WheelC 53.41      //Cm 轮毂周长 17×3.1416
#define fixWC 0.95
#define HighestSpd 1500  //rpm 
#define UpTime 100     //ms
#define DownTime 100    //ms
#define spedC 163.84f    //速度系数 60*8192/3000

#define CtrlPerid 10 //ms

#define deltaAng 25.0f   
#define window 4


double sum = 0;
double alpha[window] = {0};


//====================函数实现========================//

/**
 * @name: rx_thread
 * @msg:  接受进程函数
 * @param {void *} 设置接口的结构
 * @return: 无
 */

void * rx_thread(void *data)

{
    RX_CTX *ctx = (RX_CTX *)data;
    ctx->total = 0; // reset counter

    VCI_CAN_OBJ can[RX_BUFF_SIZE]; // buffer
    int cnt; // current received
    int i;
    int pos = 0;

    while (!ctx->stop && !ctx->error)
    {
        cnt = VCI_Receive(gDevType, gDevIdx, ctx->channel, can, RX_BUFF_SIZE, RX_WAIT_TIME);
        if (!cnt)
            continue;

        for (i = 0; i < cnt; i++) {  //每一帧

            if ( can[i].Data[1] == 0xDB ){
                if ( NeedRecData){
                    NeedRecData = 0;
                    memcpy(motor[MotorOnControl].recData , can[i].Data , 8 );
                } 
                //printf("%02x,%02x,%02x,%02x,%02x\r\n",can[i].Data[3],can[i].Data[4],can[i].Data[5],can[i].Data[6],can[i].Data[7]);
                RecRight = 1;
            }
            else if ( can[i].Data[1] == 0xFF){
                printf( "Address invalid, please check ID \n");
            }
            if ( can[i].Data[1] == 0xFE && can[i].Data[3] == 0x20){
                pos = (can[i].Data[4]<<24) | (can[i].Data[5]<<16) | (can[i].Data[6]<<8) | (can[i].Data[7]);
                if (can[i].ID < 5 ){
                    motor[can[i].ID-1].watchdog = 0;
                    motor[can[i].ID-1].odom = double(pos)/LineNum/10*3.1415926;
                    //printf("ID:%d\tpos:%3.4f\n",can[i].ID,double(pos)/LineNum/10*3.1415926);
                }
                if ( can[i].ID == 7 || can[i].ID == 8 ){
                    motor[can[i].ID-1].odom = double(pos)/409600 * WheelC;
                    //printf("ID:%d\tpos:%3.4f\n",can[i].ID,double(pos)/409600 * WheelC);
                }
      
                

                //printf("%x,%x,%x,%x\n",can[i].Data[4],can[i].Data[5],can[i].Data[6],can[i].Data[7]);
            }        
                
        }
        if (ctx->error) break;
        
    }

    printf("CAN%d RX thread terminated, %d frames received & verified: %s\n",
        ctx->channel, ctx->total, ctx->error ? "error(s) detected" : "no error");

    pthread_exit(0);

}

/**
 * @name: CanbusInit
 * @msg: CanBus设备的初始化，包括波特率，通道，接收进程
 * @param {uint} 通道，{uint} 波特率， {uint} 发送模式（正常，单次，自检）
 * @return: 错误指示
 */
int CanbusInit(unsigned Channel , unsigned Baud , unsigned TxType  ){
    gDevType = VCI_USBCAN2;
    gDevIdx = 0;  
    gChMask = Channel;
    gBaud = Baud;   //CANbps= Fpclk/((BRP+1)*((Tseg1+1)+(Tseg2+1)+1) Tseg1=Baud[14:12],Tseg2=Baud[11:8],Baud[5:0]=BRP,Fpclk=8M
    gTxType = TxType;

    config.AccCode = 0;
    config.AccMask = 0xffffffff;
    config.Filter = 1;
    config.Mode = 0;
    config.Timing0 = gBaud & 0xff;
    config.Timing1 = gBaud >> 8;
    printf("DevType=%d, DevIdx=%d, ChMask=0x%x, Baud=0x%04x, TxType=%d\n",
        gDevType, gDevIdx, gChMask, gBaud, gTxType);

    if (!VCI_OpenDevice(gDevType, gDevIdx, 0)) {
        printf("VCI_OpenDevice failed\n");
        return 0;
    }
    printf("VCI_OpenDevice succeeded\n");

    // ----- init & start -------------------------------------------------

    int i, j;
    for (i = 0; i < MAX_CHANNELS; i++)
    {
        if ((gChMask & (1 << i)) == 0) continue;

        if (!VCI_InitCAN(gDevType, gDevIdx, i, &config))
        {
            printf("VCI_InitCAN(%d) failed\n", i);
            return 0;
        }
        printf("VCI_InitCAN(%d) succeeded\n", i);

        if (!VCI_StartCAN(gDevType, gDevIdx, i))
        {
            printf("VCI_StartCAN(%d) failed\n", i);
            return 0;
        }
        printf("VCI_StartCAN(%d) succeeded\n", i);
    }

  // ----- create RX-threads --------------------------------------------

     for (i = 0; i < MAX_CHANNELS; i++)
    {
        if ((gChMask & (1 << i)) == 0) continue;

        rx_ctx[i].channel = i;
        rx_ctx[i].stop = 0;
        rx_ctx[i].total = 0;
        rx_ctx[i].error = 0;

        pthread_create(&rx_threads[i], NULL, rx_thread, &rx_ctx[i]);
    }

     // ----- wait --------------------------------------------------------

    ROS_INFO("Successed! go to Finish Config: baud: t0=0x%02x, t1=0x%02x,\n",
        config.Timing0, config.Timing1);
    //getchar();

    return 1;

}

/**
 * @name: MotorInit
 * @msg: 电机结构体初始化
 * @param {void} 
 * @return: 无
 */
void MotorInit()
{
    int i = 0;
    for (i = 0; i < 8; i++)
    {
        motor[i].num = i;
        motor[i].ID = FindID[i];
        motor[i].Can = dftCan[i] - 1;
        motor[i].Pos = 0;
        motor[i].Speed = 0;
        motor[i].Mode = M_begin;
        motor[i].En = 0;
        motor[i].odom = 0.0f;
        motor[i].odom_last = 0.0f;
        motor[i].watchdog = 0;
    }

    printf("Moter Init Completed !");
}


/**
 * @name: sendCommand
 * @msg: 用来发送控制命令并接受回信，只处理一个,回传只判断是否正确与地址是否错误
 * @param {uint} 通道， {VCI_CAN_OBJ *} 数据帧指针 
 * @return: void
 */
void sendCommand( uint chan , VCI_CAN_OBJ *buff )
{
    VCI_Transmit(gDevType, gDevIdx, chan, buff, 1);
    //while( !RecRight );   //等待接收
    RecRight = 0;

    return;
}


/**
 * @name: askError
 * @msg: 询问电机错误
 * @param {Motor *} 询问的电机
 * @return: 
 */
void askError( Motor *m )
{
    VCI_CAN_OBJ can;
    can.SendType = gTxType;
    can.DataLen = 8;
    can.ID = m->ID;
    can.ExternFlag = 0;
    
    int i;
    for ( i = 0; i<8; i++){
        can.Data[i] = AskAlarm[i];
    }

    NeedRecData = 1;
    VCI_Transmit(gDevType, gDevIdx, m->Can , &can, 1);

    //while( !RecRight );   //等待接收
    RecRight = 0;

    m->Error = 1;
    //printf("Motor %d ",m->num);

    if ( m->recData[3] == 0xE3 )
    {
        switch ( m->recData[7])
        {
            case 0x80: case 0x00:  /*printf("State : 停机状态");*/ m->Error = 0; break;
            case 0x81: case 0x01:  /*printf("State : 启动状态");*/ m->Error = 0; break;
        /*    case 0x82: case 0x02:  printf("State : 过流"); break;
            case 0x84: case 0x04:  printf("State : 过压"); break;
            case 0x88: case 0x08:  printf("State : 编码器故障"); break;
            case 0x90: case 0x10:  printf("State : 过热"); break;
            case 0xA0: case 0x20:  printf("State : 欠压"); break;
            case 0xC0: case 0x40:  printf("State : 过载"); break;*/
        }
    }else
    {
        //printf(" Receive Error!");
    }
   // printf("\r\n");
/*
    if (m->num == 0 || m->num == 4)
        printf("\r\n");
    else 
        printf(" | ");
        */
}

/**
 * @name: clrError
 * @msg: 清除报警
 * @param {Motor *} 处理的电机 
 * @return: 
 */
void clrError( Motor *m)
{
    VCI_CAN_OBJ can;
    can.SendType = gTxType;
    can.DataLen = 8;
    can.ID = m->ID;
    can.ExternFlag = 0;
    
    int i;
    for ( i = 0; i<8; i++){
        can.Data[i] = ClrAlarm[i];
    }

    VCI_Transmit(gDevType, gDevIdx, m->Can , &can, 1);

    //while( !RecRight );   //等待接收
    RecRight = 0;

}

void askPos( Motor *m )
{
    VCI_CAN_OBJ can;
    can.SendType = gTxType;
    can.DataLen = 8;
    can.ID = m->ID;
    can.ExternFlag = 0;
    
    int i;
    int recPos = 0;
    for ( i = 0; i<8; i++){
        can.Data[i] = AskPos[i];
    }

    NeedRecData = 1;
    VCI_Transmit(gDevType, gDevIdx, m->Can , &can, 1);

    //while( !RecRight );   //等待接收
    RecRight = 0;

   
    if ( m->recData[3] == 0xE8 )
    {
        for (i = 4;i<8;i++)
        {
            recPos += m->recData[i]*( 1<<( ( 7-i ) * 8) );
        }
    }else
    {
        printf(" Receive Error!");
    }
    printf("Motor%d: PosNow: %3.2f", m->ID , recPos * 360.f / 20/ LineNum);
    printf("\r\n");
}

void writeCurrent( Motor *m)
{
    VCI_CAN_OBJ can;
    can.SendType = gTxType;
    can.DataLen = 8;
    can.ID = m->ID;
    can.ExternFlag = 0;
    
    int i;
    for ( i = 0; i<8; i++){
        can.Data[i] = WriteI[i];
    }

    VCI_Transmit(gDevType, gDevIdx, m->Can , &can, 1);

    //while( !RecRight );   //等待接收
    RecRight = 0;

}


/**
 * @name: ctlMotor
 * @msg: 控制电机的底层，一次只控制一个电机
 * @param {Motor *} 电机类，指定哪个电机， {uint} 控制模式（速度和位置）， {int} 速度或位置值， {bool} 使能，用于开关电机  
 * @return: void
 */
void ctlMotor(Motor *m , uint mode , float data , bool Enb )
{
    VCI_CAN_OBJ can;
    can.SendType = gTxType;
    can.DataLen = 8;
    can.ID = m->ID;
    can.ExternFlag = 0;

    int i;
    int speed = 0;
    int position = 0;

    MotorOnControl = m->num;  //控制回传数据
    
    //askError( m );
    //if ( m->Error )
        //clrError( m );

    can.Data[0] = 0;
    can.Data[1] = 0xDA;
    can.Data[2] = 0;
    can.Data[3] = 0;
    can.Data[4] = 0;
    can.Data[5] = 0;
    can.Data[6] = 0;
    can.Data[7] = 0;

    if ( Enb != m->En)    //如果使能更新
    {
        m->En = Enb;
        if ( Enb == 1 )
        {
            can.Data[3] = Enable[3];
            can.Data[7] = Enable[7];
        }
        else
        {
            can.Data[3] = Stop[3];
            can.Data[7] = Stop[7];
        }
        sendCommand( m->Can , &can);

   /*     can.Data[3] = WriteI[3];
        can.Data[6] = WriteI[6];
        can.Data[7] = WriteI[7];
        sendCommand( m->Can , &can);*/
    }

    if ( mode != m->Mode )    //如果模式更新
    {
        m->Mode = mode;
        if ( mode == M_pos)
        {
            can.Data[3] = PosMode[3];
            can.Data[7] = PosMode[7];
            sendCommand( m->Can , &can);    //设置位置模式
            //printf("pos mode1\n");
            
            can.Data[3] = PosAbslt[3];
            can.Data[7] = PosAbslt[7];
            sendCommand( m->Can , &can);    //设置绝对位置模式

            can.Data[3] = SetPosSpd[3];
            for ( i = 4; i<8; i++){
                can.Data[i] = ( int(HighestSpd*2.73) >> (7 - i)*8 ) & 0xff;
            }
            sendCommand( m->Can , &can);    //设置位置最高速

            can.Data[3] = SetUpDnTime[3];
            can.Data[4] = 0x00;
            can.Data[5] = 0x00;
            can.Data[6] = int(UpTime/100);
            can.Data[7] = int(DownTime/100);
            sendCommand( m->Can , &can);    //设置加减速时间

        }
        else if ( mode == M_spd)
        {
            
            can.Data[3] = SpdMode[3];
            can.Data[7] = SpdMode[7];
            sendCommand( m->Can  , &can); 

             can.Data[3] = 0x13;
            can.Data[4] = 0x00;
            can.Data[5] = 0x00;
            can.Data[6] = int(UpTime/100);
            can.Data[7] = int(DownTime/100);
            sendCommand( m->Can , &can);    //设置加减速时间

           
        }

    }

    if ( mode == M_spd )
    {
        speed = (int) ( data * spedC / (WheelC*fixWC) );
        can.Data[3] = SetSpd[3]; 
        for ( i = 4; i<8; i++){
            can.Data[i] = ( speed >> (7 - i)*8 ) & 0xff;
        }
        sendCommand( m->Can , &can); 
    } 
    else if ( mode == M_pos )
    {
        
        position = (int) ( data * LineNum * 20 /360.0f );   //20为减速比
        can.Data[3] = SetPos[3]; 
        for ( i = 4; i<8; i++){
            can.Data[i] = ( position >> (7 - i)*8 ) & 0xff;
        }
        sendCommand( m->Can , &can); 
    }

    
}

/**
 * @name: setMotorPID
 * @msg: 设置电机PID
 * @param {Motor *} 某个电机， {int} P,I,D 
 * @return: void
 */
void setMotorPID( Motor *m, BYTE mode, int P , int I , int D)
{

    if (mode != M_pos && mode != M_spd ) 
        return;

    VCI_CAN_OBJ can;
    can.SendType = gTxType;
    can.DataLen = 8;
    can.ID = m->ID;
    can.ExternFlag = 0;

    int  i;

    can.Data[1] = 0xDA;
    can.Data[3] = mode == M_pos ? 0x20 : 0x23 ; 
    for ( i = 4; i<8; i++){
        can.Data[i] = ( P >> (7 - i)*8 ) & 0xff;
    }
    sendCommand( m->Can , &can);        //改变P

    for ( i = 4; i<8; i++){
        can.Data[i] = ( I >> (7 - i)*8 ) & 0xff;
    }
    can.Data[3] ++ ;            
    sendCommand( m->Can , &can);        //功能字加1,改变I

    for ( i = 4; i<8; i++){
        can.Data[i] = ( D >> (7 - i)*8 ) & 0xff;
    }
    can.Data[3] ++ ;            
    sendCommand( m->Can , &can);        //功能字加1,改变D

}

void stop(int n)
{
    int i;
    VCI_CAN_OBJ can;
    can.SendType = gTxType;
    can.ID = motor[n].ID;
    can.DataLen = 8;
    can.ExternFlag = 0;

    for (i = 0; i < 8; i++){
        can.Data[i] = EmgStop[i];
    }
    sendCommand(motor[n].Can , &can);
}

void dealDualSolution( double &m_p , double &p_last , int8_t &m_dir)
{
    if ( m_p > 180.0f - DegreeLmt ) { m_p -= 180.0f; m_dir = -1 ;}
    else if ( m_p < DegreeLmt - 180.0f ) { m_p += 180.0f; m_dir = -1 ;}
    else if ( m_p <= DegreeLmt && m_p >= -DegreeLmt ){ m_dir =1; }
    else {
        if ( fabs( m_p -p_last ) > 90.0f  ){ m_p += (m_p > 0 ? -180 : 180); m_dir = -1; }//stop(4);stop(5);stop(6);stop(7);}
        else m_dir = 1;
    }

    if ( m_p - p_last >deltaAng ){
        m_p = p_last + deltaAng;

    }else if ( m_p - p_last < -deltaAng ){
        m_p = p_last - deltaAng;
    }
    p_last = m_p;
}


/**
 * @name: dealCommand
 * @msg: 处理上层指令
 * @param {double} 车行进轴速度 cm/s， {double} 车侧移轴速度 cm/s ， {double} 车自旋转速度 rad/s
 * @return: void
 */
void dealCommand( double x, double y, double z )
{
    
    //TODO ：需要完成车体运动建模后编写，调用底层每个电机的驱动

    x = fabs(x) > 0.1f ? x : 0;
    y = fabs(y) > 0.1f ? y : 0;
    z = fabs(z) > 0.01f ? z : 0;

    double zB = z * B;
    double zA = z * A;

    double x3 = x + zB;
    double y3 = y + zA;
    double x2 = x + zB;
    double y2 = y - zA;
    double x1 = x - zB;
    double y1 = y + zA;
    double x0 = x - zB;
    double y0 = y - zA;
    
    double m3p = 57.29578f * atan2( x3, y3 );
    double m2p = 57.29578f * atan2( x2, y2 );
    double m1p = 57.29578f * atan2( x1, y1 );
    double m0p = 57.29578f * atan2( x0, y0 );
    
    if ( x!=0 || y!=0 || z!=0 ){
        dealDualSolution( m3p , m3p_last , m3dir);
        dealDualSolution( m2p , m2p_last , m2dir);
        dealDualSolution( m1p , m1p_last , m1dir);
        dealDualSolution( m0p , m0p_last , m0dir);

        motor[0].plan.push(m0p);
        motor[1].plan.push(m1p);
        motor[2].plan.push(m2p);
        motor[3].plan.push(m3p);

        // ctlMotor( &motor[3] , M_pos , m3p , true );   
        // ctlMotor( &motor[2] , M_pos , m2p , true );
        // ctlMotor( &motor[1] , M_pos , m1p , true );   
        // ctlMotor( &motor[0] , M_pos , m0p , true );
    }

    motor[4].plan.push( m0dir * sqrt (x0 * x0 + y0 * y0 ));
    motor[5].plan.push(-m1dir * sqrt (x1 * x1 + y1 * y1 ));
    motor[6].plan.push( m2dir * sqrt (x2 * x2 + y2 * y2 ));
    motor[7].plan.push(-m3dir * sqrt (x3 * x3 + y3 * y3 ));

    
  
    for( int i = 0; i<8 ; i++ ){
        std::queue<double> tmp;
        double p1 = 0;
        double p2 = 0;
        double p01 = 0;
        double pf1 = 0;
        double p02 = 0;
        double pf2 = 0;
        double v1 = 0;
        double v2 = 0;
        double tf = 0.05f;
        // printf("%d,\n",motor[i].plan.size());
        if (motor[i].plan.size() > window ){  //保持window长度的队列
            motor[i].plan.pop();

            for (int j = 0; j<window; j++){  //遍历列表取数
                tmp.push(motor[i].plan.front());
                if ( j < window - 1 ){
                    p1 += tmp.back()*alpha[j]/sum;
                }
                if (j >= 1){
                    p2 += tmp.back()*alpha[j-1]/sum;
                }
                if(j == 0) p01 = tmp.back();
                if(j == 1) p02 = tmp.back();
                if (j == window - 1) pf2 = tmp.back();
                if (j == window - 2) pf1 = tmp.back(); 
                motor[i].plan.pop();
            }
            while(!tmp.empty()){        //还原原队列
                motor[i].plan.push(tmp.front());
                tmp.pop();
            }
            v1 = ( pf1 - p01 ) / ( (window - 2) * tf);  //固定频率为20Hz
            v2 = ( pf2 - p02 ) / ( (window - 2) * tf);  //固定频率为20Hz
            motor[i].plan_param[0] = p1;
            motor[i].plan_param[1] = v1;
            motor[i].plan_param[2] = 3/(tf*tf)*(p2 - p1) -2/tf*v1 -1/tf*v2;
            motor[i].plan_param[3] = -2/(tf*tf*tf)*(p2 - p1) +1/(tf*tf)*(v2 + v1);
            motor[i].run_time = 0;
            motor[i].run_max_time = tf;
        
        }
    }
    
    // ctlMotor( &motor[7] , M_spd , -m3dir * sqrt (x3 * x3 + y3 * y3 ), true );   
    // ctlMotor( &motor[6] , M_spd , m2dir * sqrt (x2 * x2 + y2 * y2 ), true );
    // ctlMotor( &motor[5] , M_spd , -m1dir * sqrt (x1 * x1 + y1 * y1 ), true );   
    // ctlMotor( &motor[4] , M_spd , m0dir * sqrt (x0 * x0 + y0 * y0 ), true );

    //ROS_INFO("%f\n",ros::Time().now().toSec());

}

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const geometry_msgs::Twist& cmd_vel)
{

	dealCommand( -cmd_vel.linear.y*100 , cmd_vel.linear.x*100 , 1.25*cmd_vel.angular.z );  // x:y: cm/s ; z: rad/s ;
    //askPos(&motor[2]);

    //ROS_INFO("\nx: %1.4f \r\ny: %1.4f \r\nz: %1.4f \r\n",cmd_vel.linear.y,cmd_vel.linear.x,cmd_vel.angular.z); 
    
	//ROS_INFO("I heard:");
}

int main(int argc, char *argv[])
{

	int i = 0;
    int ch;
    bool quit = 0;
    double x , y , z = 0.0f;
    int pub_cnt=0;

    double first = 1;
    double first2, first3, first6, first7 = 0.0f;

	ros::init(argc, argv, "chassis_sub");

	ros::NodeHandle n;

	MotorInit();
	if ( !CanbusInit(CAN1 , BAUD_500K ,  NORMAL )  )  
    {
        ROS_INFO("Please Recheck Devices! Config failed. \n");
        return 0;
    }

	ros::Subscriber sub = n.subscribe("cmd_vel", 1000, chatterCallback);
    ros::Publisher pub = n.advertise<geometry_msgs::TwistStamped>("encoder",20);
	
     ros::Rate loop_rate( int(1000 / CtrlPerid) );
     
     geometry_msgs::TwistStamped en;

     for (int j = 0; j<window-1; j++){  //初始化参数
           
            //用钟形分段直线来计算权重系数
            if ( j < int((window - 1)/2) ){
                alpha[j] = 1+3*j;
            }
            else if ( j > int((window - 1)/2)){
                alpha[j] = 1+3*(window - j -2);
            }
            else {
                if ( (window - 1)%2 == 0){
                    alpha[j] = alpha[j-1];
                }
                else{
                   alpha[j] = 1+3*j;
                }
            }
            sum += alpha[j];
     }
double sum = 0;
double err_last[8] = {0};
     while(ros::ok())
     {

        for ( i = 0; i<8; i++){
            
            double now = motor[i].plan_param[0] + motor[i].plan_param[1]*motor[i].run_time + motor[i].plan_param[2]*motor[i].run_time*motor[i].run_time + motor[i].plan_param[3]*motor[i].run_time*motor[i].run_time*motor[i].run_time;
            if ( i < 4 ){
                ROS_INFO("%d",motor[i].watchdog);//这里必须打印，如果不打印有问题
              
                double err = now - motor[i].odom*57.29578f;
                //if (i == 0) ROS_INFO("%f",err);
               
                now = 30 * err  + 1.0 * (err - err_last[i]);
                err_last[i] = err;

                motor[i].watchdog ++;
                if ( motor[i].watchdog > 8 ){
                    motor[i].watchdog --;
                    clrError( &motor[i] );
                    stop(i);
                }
                else{
                    ctlMotor( &motor[i] , M_spd , now , true );  //PID有可能需要限幅
                }
            }
            else{
                ctlMotor( &motor[i] , M_spd , now , true );
            }

            if ( motor[i].run_time >= motor[i].run_max_time ) {
                motor[i].run_time = motor[i].run_max_time;
            }else{
                 motor[i].run_time += double(CtrlPerid / 1000.0f );

            }

        //if (i==0) ROS_INFO("%f",now);
                   
        }
        //printf("%f\n",now);

         if (first == 1){//去除上电之后未运行里程计的误差，去掉初始值
             first7 = motor[7].odom;
             first6 = motor[6].odom;
             first3 = motor[3].odom;
             first2 = motor[2].odom;
             first = 0;
         }
            en.header.stamp = ros::Time().now();
            en.twist.linear.x = motor[7].odom - first7;
            en.twist.linear.y = motor[6].odom - first6;
            en.twist.angular.x = motor[3].odom - first3;
            en.twist.angular.y = motor[2].odom - first2;
            pub.publish(en);

            ros::spinOnce();
            loop_rate.sleep();

     }

	for (i = 0; i < MAX_CHANNELS; i++)
    {
        if ((gChMask & (1 << i)) == 0) continue;

        rx_ctx[i].stop = 1;

        pthread_join(rx_threads[i], NULL);

    }

    VCI_CloseDevice(gDevType, gDevIdx);
    ROS_INFO("VCI_CloseDevice\n");

	return 0;
}
