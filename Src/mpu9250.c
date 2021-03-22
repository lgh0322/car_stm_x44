#include "mpu9250.h"
#include "usart.h" 
#include "inv_mpu_dmp_motion_driver.h"
#include "inv_mpu.h"
#include "i2c.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32开发板
//MPU9250驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2015/12/29
//版本：V1.1
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved			
//********************************************************************************
//修改说明
//20160905 V1.1
//9250集成的6500的ID可能有2个,将MPU6500_ID改为MPU6500_ID1和MPU6500_ID2
////////////////////////////////////////////////////////////////////////////////// 	

//初始化MPU9250
//返回值:0,成功
//    其他,错误代码

signed short int gyro_offsetx=-91,gyro_offsety=-46,gyro_offsetz=-26;
float tmp1,tmp2,tmp3;
float magoffsetx=0.0754393101889353,magoffsety=-0.0357509608134262,magoffsetz=0.0242901804855749;
float B[6]={1,0,0,1,0,1};


float accoffsetx=-0.00804839165299282,accoffsety=0.00268697705921562,accoffsetz=-0.0048420261889248;
float accsensx=1.00039363094007,accsensy=1.03127954357308,accsensz=0.969287648701866;

#define filter_high 0.9
#define filter_low 	0.1
signed short int accoldx,accoldy,accoldz;
signed short int magoldx,magoldy,magoldz;
signed short int gyrooldx,gyrooldy,gyrooldz;

u8 MPU_Init(void)
{
    u8 res=0;
    MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X80);//复位MPU9250
    HAL_Delay(100);  //延时100ms
    MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X00);//唤醒MPU9250
    MPU_Set_Gyro_Fsr(0);					        	//陀螺仪传感器,±250dps
	MPU_Set_Accel_Fsr(0);					       	 	//加速度传感器,±2g
    MPU_Set_Rate(200);						       	 	//设置采样率200Hz
    MPU_Write_Byte(MPU9250_ADDR,MPU_INT_EN_REG,0X00);   //关闭所有中断
	MPU_Write_Byte(MPU9250_ADDR,MPU_USER_CTRL_REG,0X00);//I2C主模式关闭
	MPU_Write_Byte(MPU9250_ADDR,MPU_FIFO_EN_REG,0X00);	//关闭FIFO
	MPU_Write_Byte(MPU9250_ADDR,MPU_INTBP_CFG_REG,0X82);//INT引脚低电平有效，开启bypass模式，可以直接读取磁力计
    res=MPU_Read_Byte(MPU9250_ADDR,MPU_DEVICE_ID_REG);  //读取MPU6500的ID
    if(res==MPU6500_ID1||res==MPU6500_ID2) //器件ID正确
    {
        MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X01);  	//设置CLKSEL,PLL X轴为参考
        MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT2_REG,0X00);  	//加速度与陀螺仪都工作
		MPU_Set_Rate(200);						       	//设置采样率为200Hz   
    }else return 1;
 
    res=MPU_Read_Byte(AK8963_ADDR,MAG_WIA);    			//读取AK8963 ID   
    if(res==AK8963_ID)
    {
        MPU_Write_Byte(AK8963_ADDR,MAG_CNTL2,0X01);		//复位AK8963
		HAL_Delay(50);
        MPU_Write_Byte(AK8963_ADDR,MAG_CNTL1,0X11);		//设置AK8963为单次测量
			
    }else return 1;
		
//		calibrate();

    return 0;
}

//设置MPU9250陀螺仪传感器满量程范围
//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU9250_ADDR,MPU_GYRO_CFG_REG,(fsr<<3)|3);//设置陀螺仪满量程范围  
}
//设置MPU9250加速度传感器满量程范围
//fsr:0,±2g;1,±4g;2,±8g;3,±16g
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_Accel_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU9250_ADDR,MPU_ACCEL_CFG_REG,fsr<<3);//设置加速度传感器满量程范围  
}

//设置MPU9250的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_LPF(u16 lpf)
{
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte(MPU9250_ADDR,MPU_CFG_REG,data);//设置数字低通滤波器  
}

//设置MPU9250的采样率(假定Fs=1KHz)
//rate:4~1000(Hz)
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_Rate(u16 rate)
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU9250_ADDR,MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
 	return MPU_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
}


void calibrate(void)
{
	u8 t;
	signed short int gx,gy,gz,sumx=0,sumy=0,sumz=0;
	for (t=0;t<10;t++)
	{
		MPU_Get_Gyroscope(&gx,&gy,&gz);
		
		sumx=sumx+gx;
		sumy=sumy+gy;
		sumz=sumz+gz;
	}
	
	gyro_offsetx=sumx/10;
	gyro_offsety=sumy/10;
	gyro_offsetz=sumz/10;
	

}



//得到温度值
//返回值:温度值(扩大了100倍)
signed short int MPU_Get_Temperature(void)
{
    u8 buf[2]; 
    signed short int raw;
	float temp;
	MPU_Read_Len(MPU9250_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
    raw=((u16)buf[0]<<8)|buf[1];  
    temp=21+((double)raw)/333.87;  
    return temp*100;;
}
///////////////////////////////////////////////////
/////////////////////////////////
//得到陀螺仪值(原始值)，对源数据平均值滤波，并调整陀螺仪方位
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
u8 MPU_Get_Gyroscope(signed short int *gx,signed short int *gy,signed short int *gz)
{
    u8 buf[6],res; 
	res=MPU_Read_Len(MPU9250_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gx=(((u16)buf[0]<<8)|buf[1]);  
		*gy=(((u16)buf[2]<<8)|buf[3]);  
		*gz=(((u16)buf[4]<<8)|buf[5]);

		*gx-=gyro_offsetx;
		*gy-=gyro_offsety;
		*gz-=gyro_offsetz;

		*gx=(signed short int)(gyrooldx*0.2+*gx*0.8);
		*gy=(signed short int)(gyrooldy*0.2+*gy*0.8);
		*gz=(signed short int)(gyrooldz*0.2+*gz*0.8);
		gyrooldx=*gx;
		gyrooldy=*gy;
		gyrooldz=*gz;
		
	} 	
    return res;
}

u8 MPU_Get_Gyro(signed short int *igx,signed short int *igy,signed short int *igz,float *gx,float *gy,float *gz)
{
	u8 res;
	res=MPU_Get_Gyroscope(igx,igy,igz);
	if (res==0)
	{
	*gx=(float)(*igx)*gryo_scale;
	*gy=(float)(*igy)*gryo_scale;
	*gz=(float)(*igz)*gryo_scale;
		
	}
	return res;
}

//得到加速度值(原始值)，低通滤波并调整加速度计方位
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
u8 MPU_Get_Accelerometer(signed short int *ax,signed short int *ay,signed short int *az)
{
    u8 buf[6],res;  
	res=MPU_Read_Len(MPU9250_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ax=(((u16)buf[0]<<8)|buf[1]);  
		*ay=(((u16)buf[2]<<8)|buf[3]);  
		*az=(((u16)buf[4]<<8)|buf[5]);
	
		*ax=(signed short int)(accoldx*filter_high+*ax*filter_low);
		*ay=(signed short int)(accoldy*filter_high+*ay*filter_low);
		*az=(signed short int)(accoldz*filter_high+*az*filter_low);
		accoldx=*ax;
		accoldy=*ay;
		accoldz=*az;
		
	} 	
    return res;
}


u8 MPU_Get_Accel(signed short int *iax,signed short int *iay,signed short int *iaz,float *ax,float *ay,float *az)
{
	u8 res;
	res=MPU_Get_Accelerometer(iax,iay,iaz);
	if (res==0)
	{
	tmp1=(float)(*iax)*accel_scale-accoffsetx;
	tmp2=(float)(*iay)*accel_scale-accoffsety;
	tmp3=(float)(*iaz)*accel_scale-accoffsetz;
	*ax=tmp1*accsensx;
	*ay=tmp2*accsensy;
	*az=tmp3*accsensz;
	}
	return res;
}
//得到磁力计值(原始值)，平均滤波并调整方位
//mx,my,mz:磁力计x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
u8 MPU_Get_Magnetometer(signed short int *mx,signed short int *my,signed short int *mz)
{
    u8 buf[8],res;  
//				res=MPU_Read_Byte(AK8963_ADDR,0x01);
//			printf("MAG_ST1=%d  ",res);
//				res=MPU_Read_Byte(AK8963_ADDR,MAG_ST2);
//			printf("MAG_ST2=%d  ",res);

 	res=MPU_Read_Len(AK8963_ADDR,MAG_ST1,8,buf);
 
	if(res==0)
	{ 
		*my=((u16)buf[2]<<8)|buf[1];  
		*mx=((u16)buf[4]<<8)|buf[3];  
		*mz=((u16)buf[6]<<8)|buf[5];

		*mz=-*mz;

		*mx=(signed short int)(magoldx*0.5+*mx*0.5);
		*my=(signed short int)(magoldy*0.5+*my*0.5);
		*mz=(signed short int)(magoldz*0.5+*mz*0.5);
		magoldx=*mx;
		magoldy=*my;
		magoldz=*mz;
	} 	 
	HAL_Delay(1);
	MPU_Write_Byte(AK8963_ADDR,MAG_CNTL1,0X11); //AK8963每次读完以后都需要重新设置为单次测量模式

    return res;
}


u8 MPU_Get_Mag(signed short int *imx,signed short int *imy,signed short int *imz,float *mx,float *my,float *mz)
{
	u8 res;
	res=MPU_Get_Magnetometer(imx,imy,imz);
	if (res==0)
	{
	tmp1=(float)(*imx)*mag_scale-magoffsetx;
	tmp2=(float)(*imy)*mag_scale-magoffsety;
	tmp3=(float)(*imz)*mag_scale-magoffsetz;
	*mx=B[0]*tmp1+B[1]*tmp2+B[2]*tmp3;
	*my=B[1]*tmp1+B[3]*tmp2+B[4]*tmp3;
	*mz=B[2]*tmp1+B[4]*tmp2+B[5]*tmp3;
	}
	return res;
}

//IIC连续写
//addr:器件地址 
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//    其他,错误代码
u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
    HAL_I2C_Mem_Write(&hi2c1,addr,reg,1,buf,len,2000);
    return 0;
} 

//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
		HAL_I2C_Mem_Read(&hi2c1,addr,reg,1,buf,len,2000);
    return 0;       
}

//IIC写一个字节 
//devaddr:器件IIC地址
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
u8 MPU_Write_Byte(u8 addr,u8 reg,u8 data)
{
    HAL_I2C_Mem_Write(&hi2c1,addr,reg,1,&data,1,2000);
    return 0;
}

//IIC读一个字节 
//reg:寄存器地址 
//返回值:读到的数据
u8 MPU_Read_Byte(u8 addr,u8 reg)
{
		u8 res;
    HAL_I2C_Mem_Read(&hi2c1,addr,reg,1,&res,1,2000);
    return res;  
}
