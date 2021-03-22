#include "mpu9250.h"
#include "usart.h" 
#include "inv_mpu_dmp_motion_driver.h"
#include "inv_mpu.h"
#include "i2c.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32������
//MPU9250��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2015/12/29
//�汾��V1.1
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved			
//********************************************************************************
//�޸�˵��
//20160905 V1.1
//9250���ɵ�6500��ID������2��,��MPU6500_ID��ΪMPU6500_ID1��MPU6500_ID2
////////////////////////////////////////////////////////////////////////////////// 	

//��ʼ��MPU9250
//����ֵ:0,�ɹ�
//    ����,�������

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
    MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X80);//��λMPU9250
    HAL_Delay(100);  //��ʱ100ms
    MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X00);//����MPU9250
    MPU_Set_Gyro_Fsr(0);					        	//�����Ǵ�����,��250dps
	MPU_Set_Accel_Fsr(0);					       	 	//���ٶȴ�����,��2g
    MPU_Set_Rate(200);						       	 	//���ò�����200Hz
    MPU_Write_Byte(MPU9250_ADDR,MPU_INT_EN_REG,0X00);   //�ر������ж�
	MPU_Write_Byte(MPU9250_ADDR,MPU_USER_CTRL_REG,0X00);//I2C��ģʽ�ر�
	MPU_Write_Byte(MPU9250_ADDR,MPU_FIFO_EN_REG,0X00);	//�ر�FIFO
	MPU_Write_Byte(MPU9250_ADDR,MPU_INTBP_CFG_REG,0X82);//INT���ŵ͵�ƽ��Ч������bypassģʽ������ֱ�Ӷ�ȡ������
    res=MPU_Read_Byte(MPU9250_ADDR,MPU_DEVICE_ID_REG);  //��ȡMPU6500��ID
    if(res==MPU6500_ID1||res==MPU6500_ID2) //����ID��ȷ
    {
        MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X01);  	//����CLKSEL,PLL X��Ϊ�ο�
        MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT2_REG,0X00);  	//���ٶ��������Ƕ�����
		MPU_Set_Rate(200);						       	//���ò�����Ϊ200Hz   
    }else return 1;
 
    res=MPU_Read_Byte(AK8963_ADDR,MAG_WIA);    			//��ȡAK8963 ID   
    if(res==AK8963_ID)
    {
        MPU_Write_Byte(AK8963_ADDR,MAG_CNTL2,0X01);		//��λAK8963
		HAL_Delay(50);
        MPU_Write_Byte(AK8963_ADDR,MAG_CNTL1,0X11);		//����AK8963Ϊ���β���
			
    }else return 1;
		
//		calibrate();

    return 0;
}

//����MPU9250�����Ǵ����������̷�Χ
//fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU9250_ADDR,MPU_GYRO_CFG_REG,(fsr<<3)|3);//���������������̷�Χ  
}
//����MPU9250���ٶȴ����������̷�Χ
//fsr:0,��2g;1,��4g;2,��8g;3,��16g
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_Accel_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU9250_ADDR,MPU_ACCEL_CFG_REG,fsr<<3);//���ü��ٶȴ����������̷�Χ  
}

//����MPU9250�����ֵ�ͨ�˲���
//lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_LPF(u16 lpf)
{
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte(MPU9250_ADDR,MPU_CFG_REG,data);//�������ֵ�ͨ�˲���  
}

//����MPU9250�Ĳ�����(�ٶ�Fs=1KHz)
//rate:4~1000(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_Rate(u16 rate)
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU9250_ADDR,MPU_SAMPLE_RATE_REG,data);	//�������ֵ�ͨ�˲���
 	return MPU_Set_LPF(rate/2);	//�Զ�����LPFΪ�����ʵ�һ��
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



//�õ��¶�ֵ
//����ֵ:�¶�ֵ(������100��)
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
//�õ�������ֵ(ԭʼֵ)����Դ����ƽ��ֵ�˲��������������Ƿ�λ
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
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

//�õ����ٶ�ֵ(ԭʼֵ)����ͨ�˲����������ٶȼƷ�λ
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
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
//�õ�������ֵ(ԭʼֵ)��ƽ���˲���������λ
//mx,my,mz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
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
	MPU_Write_Byte(AK8963_ADDR,MAG_CNTL1,0X11); //AK8963ÿ�ζ����Ժ���Ҫ��������Ϊ���β���ģʽ

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

//IIC����д
//addr:������ַ 
//reg:�Ĵ�����ַ
//len:д�볤��
//buf:������
//����ֵ:0,����
//    ����,�������
u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
    HAL_I2C_Mem_Write(&hi2c1,addr,reg,1,buf,len,2000);
    return 0;
} 

//IIC������
//addr:������ַ
//reg:Ҫ��ȡ�ļĴ�����ַ
//len:Ҫ��ȡ�ĳ���
//buf:��ȡ�������ݴ洢��
//����ֵ:0,����
//    ����,�������
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
		HAL_I2C_Mem_Read(&hi2c1,addr,reg,1,buf,len,2000);
    return 0;       
}

//IICдһ���ֽ� 
//devaddr:����IIC��ַ
//reg:�Ĵ�����ַ
//data:����
//����ֵ:0,����
//    ����,�������
u8 MPU_Write_Byte(u8 addr,u8 reg,u8 data)
{
    HAL_I2C_Mem_Write(&hi2c1,addr,reg,1,&data,1,2000);
    return 0;
}

//IIC��һ���ֽ� 
//reg:�Ĵ�����ַ 
//����ֵ:����������
u8 MPU_Read_Byte(u8 addr,u8 reg)
{
		u8 res;
    HAL_I2C_Mem_Read(&hi2c1,addr,reg,1,&res,1,2000);
    return res;  
}
