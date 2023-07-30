#include "AnoPTv8DataDef.h"
#include "AnoPTv8.h"
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
这里声明所有参数的相关信息，包含参数的最小值，最大值，指向参数的指针，参数名称，参数介绍
注意：指向参数的指针，是int32_t*，参数必须为int32_t数据类型
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int32_t parListForTest[ANOPTV8_PARNUM];
const _st_par_info AnoParInfoList[ANOPTV8_PARNUM]=
{
	//参数最小值，参数最大值，参数值指针，参数名称，参数介绍
    {0,65535,&parListForTest[0],"PID_1_P","角速度ROL：P"},
    {0,65535,&parListForTest[1],"PID_1_I","角速度ROL：I"},
    {0,65535,&parListForTest[2],"PID_1_D","角速度ROL：D"},
    {0,65535,&parListForTest[3],"PID_2_P","角速度PIT：P"},
    {0,65535,&parListForTest[4],"PID_2_I","角速度PIT：I"},
    {0,65535,&parListForTest[5],"PID_2_D","角速度PIT：D"},
    {0,65535,&parListForTest[6],"PID_3_P","角速度YAW：P"},
    {0,65535,&parListForTest[7],"PID_3_I","角速度YAW：I"},
    {0,65535,&parListForTest[8],"PID_3_D","角速度YAW：D"},
	{-100000000,100000000,&parListForTest[9],"CAL_AOFF_X","CAL_AOFF_X"},
	{-100000000,100000000,&parListForTest[10],"CAL_AOFF_Y","CAL_AOFF_Y"},
	{-100000000,100000000,&parListForTest[11],"CAL_AOFF_Z","CAL_AOFF_Z"},
	{-100000000,100000000,&parListForTest[12],"CAL_ASEN_X","CAL_ASEN_X"},
	{-100000000,100000000,&parListForTest[13],"CAL_ASEN_Y","CAL_ASEN_Y"},
	{-100000000,100000000,&parListForTest[14],"CAL_ASEN_Z","CAL_ASEN_Z"},
	{-100000000,100000000,&parListForTest[15],"CAL_AEM1_X","CAL_AEM1_X"},
	{-100000000,100000000,&parListForTest[16],"CAL_AEM1_Y","CAL_AEM1_Y"},
	{-100000000,100000000,&parListForTest[17],"CAL_AEM1_Z","CAL_AEM1_Z"},
	{-100000000,100000000,&parListForTest[18],"CAL_AEM2_X","CAL_AEM2_X"},
	{-100000000,100000000,&parListForTest[19],"CAL_AEM2_Y","CAL_AEM2_Y"},
	{-100000000,100000000,&parListForTest[20],"CAL_AEM2_Z","CAL_AEM2_Z"},
	{-100000000,100000000,&parListForTest[21],"CAL_AEM3_X","CAL_AEM3_X"},
	{-100000000,100000000,&parListForTest[22],"CAL_AEM3_Y","CAL_AEM3_Y"},
	{-100000000,100000000,&parListForTest[23],"CAL_AEM3_Z","CAL_AEM3_Z"},
	{-100000000,100000000,&parListForTest[24],"CAL_MOFF_X","CAL_MOFF_X"},
	{-100000000,100000000,&parListForTest[25],"CAL_MOFF_Y","CAL_MOFF_Y"},
	{-100000000,100000000,&parListForTest[26],"CAL_MOFF_Z","CAL_MOFF_Z"},
	{-100000000,100000000,&parListForTest[27],"CAL_MSEN_X","CAL_MSEN_X"},
	{-100000000,100000000,&parListForTest[28],"CAL_MSEN_Y","CAL_MSEN_Y"},
	{-100000000,100000000,&parListForTest[29],"CAL_MSEN_Z","CAL_MSEN_Z"},
};
uint16_t AnoPTv8GetParCount(void)
{
	return ANOPTV8_PARNUM;
}




/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
这里声明不同命令对应的函数，收到命令后，通过函数指针，自动调用对应的函数
注意，所有函数应使用相同的参数，不能更改
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void AnoPTv8CmdFun_UserCmd1(const _un_frame_v8 *p);
static void AnoPTv8CmdFun_UserCmd2(const _un_frame_v8 *p);
static void AnoPTv8CmdFun_UserCmd3(const _un_frame_v8 *p);
static void AnoPTv8CmdFun_UserCmd4(const _un_frame_v8 *p);
static void AnoPTv8CmdFun_UserCmd5(const _un_frame_v8 *p);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
命令信息定义，注意，所有命令均应在此初始化,每个命令均应设置名称，信息，和对应的执行函数，参考匿名V8协议手册
VAL0=0x00，表示本命令不需要参数，此时VAL0-VAL7均=0
VAL0!=0x00，表示有一个参数，此时VAL0表示第一个参数的数据类型，定义如下：
1：U8；
2：S8
3：U16；
4：S16；
5：U32；
6：S32；
7：Float
VAL1-VAL7意义同上,最多8字节，单字节参数最多8个，双字节参数最多4个，依此类推，参数总长度不超8字节
举例：
1、飞行模式选择命令，有一个参数，参数类型是U8，则：		{0x01,	0x01,	0x01,	0x01,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00}
2、一键起飞命令，有一个参数，参数类型是U16，则：		{0x10,	0x00,	0x05,	0x02,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00}
3、目标位置命令，有两个参数，参数类型为S32，则：		{0x10,	0x01,	0x01,	0x05,	0x05,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00}
4、平移指令，有三个参数，参数类型均为U16，则：			{0x10,	0x02,	0x03,	0x02,	0x02,	0x02,	0x00,	0x00,	0x00,	0x00,	0x00}
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const _st_cmd_info AnoCmdInfoList[ANOPTV8_CMDNUM] =
{
    //CID0	CID1	CID2	VAL0	VAL1	VAL2	VAL3	VAL4	VAL5	VAL6	VAL7	CmdName			CmdInfo				CmdFun
    {{0x01,	0xA0,	0x01,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00},	"USERCMD1",		"用户命令1",		AnoPTv8CmdFun_UserCmd1},
    {{0x01,	0xA0,	0x02,	0x01,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00},	"USERCMD2",		"用户命令2:U8",		AnoPTv8CmdFun_UserCmd2},
    {{0x01,	0xA0,	0x03,	0x02,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00},	"USERCMD3",		"用户命令3:U16",	AnoPTv8CmdFun_UserCmd3},
    {{0x01,	0xA0,	0x04,	0x03,	0x03,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00},	"USERCMD4",		"用户命令4:S16",	AnoPTv8CmdFun_UserCmd4},
    {{0x01,	0xA0,	0x05,	0x05,	0x05,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00},	"USERCMD5",		"用户命令5:U32",	AnoPTv8CmdFun_UserCmd5},
};
uint16_t AnoPTv8GetCmdCount(void)
{
	return ANOPTV8_CMDNUM;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
所有命令的执行函数的实现部分，每个命令对应一个执行函数，注意函数参数必须保持一致，不能改动
p->frame.data的前3字节分别是cmdid0到2，参数从第四字节开始
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "Ano_Parameter.h"
static void AnoPTv8CmdFun_UserCmd1(const _un_frame_v8 *p)
{
  for(u8 i = 0; i < 3; i++) {
    save.acc_offset[i] = ((float)parListForTest[9+i])/100000.0f;
    save.mag_offset[i] = ((float)parListForTest[24+i])/100000.0f;
  }
    //用户示例命令，本命令根据定义，不含参数
	AnoPTv8SendStr(p->frame.sdevid, LOG_COLOR_DEFAULT, "save offset!");
	
	data_save();
}
static void AnoPTv8CmdFun_UserCmd2(const _un_frame_v8 *p)
{
    //用户示例命令，本命令根据定义，本命令包含一个U8型参数
    uint8_t cmdval1 = p->frame.data[3];
	AnoPTv8SendValStr(p->frame.sdevid, cmdval1, "UserFunc2Run!");
}
static void AnoPTv8CmdFun_UserCmd3(const _un_frame_v8 *p)
{
    //用户示例命令，本命令根据定义，本命令包含一个U16型参数
    uint16_t cmdval1 = *(uint16_t *)(p->frame.data+3);
	AnoPTv8SendValStr(p->frame.sdevid, cmdval1, "UserFunc3Run!");
}
static void AnoPTv8CmdFun_UserCmd4(const _un_frame_v8 *p)
{
    //用户示例命令，本命令根据定义，本命令包含一个S16型参数
    int16_t cmdval1 = *(int16_t *)(p->frame.data+3);
	AnoPTv8SendValStr(p->frame.sdevid, cmdval1, "UserFunc4Run!");
}
static void AnoPTv8CmdFun_UserCmd5(const _un_frame_v8 *p)
{
    //用户示例命令，本命令根据定义，本命令包含一个U32型参数
    uint32_t cmdval1 = *(uint32_t *)(p->frame.data+3);
	AnoPTv8SendValStr(p->frame.sdevid, cmdval1, "UserFunc5Run!");
}


