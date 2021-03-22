/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "mpu9250.h"
#include "math.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "inv_mpu.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define ROLL     0
#define PITCH    1
#define YAW      2

#define XAXIS    0
#define YAXIS    1
#define ZAXIS    2
#define SQR(x)  ((x) * (x))
#define Kp 5.0f                       // proportional gain governs rate of convergence toaccelerometer/magnetometer
	 //Kp比例增益 决定了加速度计和磁力计的收敛速度
#define Ki 0.002f          // integral gain governs rate of convergenceof gyroscope biases
		//Ki积分增益 决定了陀螺仪偏差的收敛速度
     // half the sample period  
		//halfT采样周期的一半
	
static float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
static float exInt = 0, eyInt = 0, ezInt = 0; 
float init_ax, init_ay, init_az, init_gx, init_gy, init_gz, init_mx, init_my, init_mz;
//static signed short int turns=0;
//static float newdata=0.0f,olddata=0.0f;

//static float pitchoffset,rolloffset,yawoffset;

static float halfT=0;
static float k10=0.0f,k11=0.0f,k12=0.0f,k13=0.0f;
static float k20=0.0f,k21=0.0f,k22=0.0f,k23=0.0f;
static float k30=0.0f,k31=0.0f,k32=0.0f,k33=0.0f;
static float k40=0.0f,k41=0.0f,k42=0.0f,k43=0.0f;

float pitch,roll,yaw;
float invSqrt(float number);
void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,float *roll,float *pitch,float *yaw);
void CountTurns(float *newdata,float *olddata,signed short int *turns);
void CalYaw(float *yaw,signed short int *turns);
void CalibrateToZero(void);
void init_quaternion(void);
uint16_t sphereFit(float    d[][3],
                   uint16_t N,
                   uint16_t MaxIterations,
                   float    Err,
                   uint16_t Population[][3],
                   float    SphereOrigin[],
                   float     *SphereRadius);
									 
									 
									 
									 
									 
									 
									 
									 
const float x=2.718f;
const uint32_t addr = 0x0800FC00;


void writeFlashTest(float a, float b, float c)
{
  HAL_FLASH_Unlock();

	FLASH_EraseInitTypeDef f;
	f.TypeErase = FLASH_TYPEERASE_PAGES;
	f.PageAddress = addr;
	f.NbPages = 1;

	uint32_t PageError = 0;

	HAL_FLASHEx_Erase(&f, &PageError);


	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, *((uint32_t *)&x));
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr+4, *((uint32_t *)&a));
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr+8, *((uint32_t *)&b));
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr+12, *((uint32_t *)&c));
	

  HAL_FLASH_Lock();
}


float chexx(unsigned int a)
{
  float temp = *(__IO float*)(addr+a);
	return temp;
}							

float chex(void)
{
  float temp = *(__IO float*)(addr);
	return temp;
}		
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
	uint16_t acd[1];
	int sr=0;
	int dis=10000;
	int trip=0;

									 
									 
									 
	float    d[200][3];     
    float    sphereOrigin[3];
    float    sphereRadius;
   uint16_t population[2][3];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int fputc(int ch,FILE *f)
{
    uint8_t temp[1]={ch};
    HAL_UART_Transmit(&huart2,temp,1,2);
		return 0;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	struct int_param_s int_param;
	
//	signed short int test;

	signed short int igx,igy,igz;
	signed short int iax,iay,iaz;
	signed short int imx,imy,imz;
	float gx,gy,gz;
	float ax,ay,az;
	float mx,my,mz;
	int k;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
	
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
	HAL_ADC_Start_DMA(&hadc1,(uint32_t *)acd,1);
	HAL_TIM_Base_Start_IT(&htim1); 


		mpu_init(&int_param);
	 mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
	  MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X01);
	mpu_set_gyro_fsr(250);
	 mpu_set_accel_fsr(2);
	 mpu_set_lpf(98);
	 mpu_set_sample_rate(1000);
	mpu_set_bypass(1); 
 HAL_Delay(200);
 if(chex()==x){
		magoffsetx = chexx(4);
		magoffsety = chexx(8);
		magoffsetz = chexx(12);	 
 }else{
	   for(k=0;k<200;k++){
	 		MPU_Get_Gyro(&igx,&igy,&igz,&gx,&gy,&gz);
			MPU_Get_Accel(&iax,&iay,&iaz,&ax,&ay,&az);
			MPU_Get_Mag(&imx,&imy,&imz,&mx,&my,&mz);
			d[k][XAXIS] = mx;
      d[k][YAXIS] = my;
      d[k][ZAXIS] = mz;
			HAL_Delay(100);
		}
			sphereFit(d, 200, 100, 0.0f, population, sphereOrigin, &sphereRadius);
			magoffsetx = sphereOrigin[XAXIS];
			magoffsety = sphereOrigin[YAXIS];
			magoffsetz = sphereOrigin[ZAXIS];
			writeFlashTest(magoffsetx,magoffsety,magoffsetz);
 }

 
 	HAL_UART_Receive_IT(&huart2,(uint8_t *)&value,1);
  HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim4);
 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
				MPU_Get_Gyro(&igx,&igy,&igz,&gx,&gy,&gz);
				MPU_Get_Accel(&iax,&iay,&iaz,&ax,&ay,&az);
				MPU_Get_Mag(&imx,&imy,&imz,&mx,&my,&mz);
	
			AHRSupdate(gx,gy,gz,ax,ay,az,mx,my,mz,&roll,&pitch,&yaw);

		mx=GET_HALFNOWTIME();

		HAL_Delay(5);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

float invSqrt(float number)
{
	long i;
	float x,y;
	const float f=1.5f;
	
	x=number*0.5f;
	y=number;
	i=*((long*)&y);
	i=0x5f375a86-(i>>1);
	y=*((float *)&i);
	y=y*(f-(x*y*y));
	return y;
}



void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,float *roll,float *pitch,float *yaw)
{
           float norm;									//用于单位化
           float hx, hy, hz, bx, bz;		//
           float vx, vy, vz, wx, wy, wz; 
           float ex, ey, ez;
//					 float tmp0,tmp1,tmp2,tmp3;
 
           // auxiliary variables to reduce number of repeated operations  辅助变量减少重复操作次数
           float q0q0 = q0*q0;
           float q0q1 = q0*q1;
           float q0q2 = q0*q2;
           float q0q3 = q0*q3;
           float q1q1 = q1*q1;
           float q1q2 = q1*q2;
           float q1q3 = q1*q3;
           float q2q2 = q2*q2;
           float q2q3 = q2*q3;
           float q3q3 = q3*q3;
          
			
           // normalise the measurements  对加速度计和磁力计数据进行规范化
           norm = invSqrt(ax*ax + ay*ay + az*az);
           ax = ax * norm;
           ay = ay * norm;
           az = az * norm;
           norm = invSqrt(mx*mx + my*my + mz*mz);
           mx = mx * norm;
           my = my * norm;
           mz = mz * norm;
//          
//					printf("%0.2f %0.2f %0.2f ",mx,my,mz);

				
//					printf("hx=%0.2f hy=%0.2f hz=%0.2f ",hx,hy,hz);
//					printf("q0=%0.2f q1=%0.2f q2=%0.2f q3=%0.2f ",q0,q1,q2,q3);
// estimated direction of gravity and magnetic field (v and w)  //估计重力和磁场的方向
//vx,vy,vz是重力加速度在物体坐标系的表示
           vx = 2.0f*(q1q3 - q0q2);
           vy = 2.0f*(q0q1 + q2q3);
           vz = q0q0 - q1q1 - q2q2 + q3q3;
					 
					 
					
					 
					            // compute reference direction of magnetic field  计算磁场的参考方向
					 //hx,hy,hz是mx,my,mz在参考坐标系的表示
           hx = 2.0f*mx*(0.5f - q2q2 - q3q3) + 2.0f*my*(q1q2 - q0q3) + 2.0f*mz*(q1q3 + q0q2);
           hy = 2.0f*mx*(q1q2 + q0q3) + 2.0f*my*(0.5f - q1q1 - q3q3) + 2.0f*mz*(q2q3 - q0q1);
           hz = 2.0f*mx*(q1q3 - q0q2) + 2.0f*my*(q2q3 + q0q1) + 2.0f*mz*(0.5f - q1q1 -q2q2);    
						//bx,by,bz是地球磁场在参考坐标系的表示
           bx = sqrtf((hx*hx) + (hy*hy));
           bz = hz;

//						printf("%f %f ",bx,bz);
					 //wx,wy,wz是地磁场在物体坐标系的表示
           wx = 2.0f*bx*(0.5f - q2q2 - q3q3) + 2.0f*bz*(q1q3 - q0q2);
           wy = 2.0f*bx*(q1q2 - q0q3) + 2.0f*bz*(q0q1 + q2q3);
           wz = 2.0f*bx*(q0q2 + q1q3) + 2.0f*bz*(0.5f - q1q1 - q2q2); 

				
////          
//					printf("wx=%0.2f wy=%0.2f wz=%0.2f mx=%0.2f my=%0.2f mz=%0.2f ",wx,wy,wz,mx,my,mz);
// error is sum ofcross product between reference direction of fields and directionmeasured by sensors 
//ex,ey,ez是加速度计与磁力计测量出的方向与实际重力加速度与地磁场方向的误差，误差用叉积来表示，且加速度计与磁力计的权重是一样的
           ex = (ay*vz - az*vy) + (my*wz - mz*wy);
           ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
           ez = (ax*vy - ay*vx) + (mx*wy - my*wx);
// ez = (mx*wy - my*wx);
//					 ex = (ay*vz - az*vy);
//           ey = (az*vx - ax*vz);
//           ez = (ax*vy - ay*vx) + (mx*wy - my*wx);
//					printf("eaz=%0.2f emz=%0.2f  ",(ax*vy - ay*vx),(mx*wy - my*wx));
           // integral error scaled integral gain
					 //积分误差

				halfT=GET_HALFNOWTIME();
			
//			if(ex != 0.0f && ey != 0.0f && ez != 0.0f) 
			{
           exInt = exInt + ex*halfT;
           eyInt = eyInt + ey*halfT;
           ezInt = ezInt + ez*halfT;
				
					// printf("exInt=%0.1f eyInt=%0.1f ezInt=%0.1f ",exInt,eyInt,ezInt);
           // adjusted gyroscope measurements
					 //PI调节陀螺仪数据
           gx = gx + Kp*ex + Ki*exInt;
           gy = gy + Kp*ey + Ki*eyInt;
           gz = gz + Kp*ez + Ki*ezInt;
				
			}
//					 printf("x=%0.1f y=%0.1f z=%0.1f	",Kp*ex + exInt,Kp*ey + eyInt,Kp*ez + ezInt);
          
           // integrate quaernion rate aafnd normalaizle
					 //欧拉法解微分方程

//        
//           tmp0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
//           tmp1 = q1 + ( q0*gx + q2*gz - q3*gy)*halfT;
//           tmp2 = q2 + ( q0*gy - q1*gz + q3*gx)*halfT;
//           tmp3 = q3 + ( q0*gz + q1*gy - q2*gx)*halfT; 
//					 q0=tmp0;
//					 q1=tmp1;
//					 q2=tmp2;
//					 q3=tmp3;
					 //printf("q0=%0.1f q1=%0.1f q2=%0.1f q3=%0.1f",q0,q1,q2,q3);
////RUNGE_KUTTA 法解微分方程
					  k10=0.5 * (-gx*q1 - gy*q2 - gz*q3);
						k11=0.5 * ( gx*q0 + gz*q2 - gy*q3);
						k12=0.5 * ( gy*q0 - gz*q1 + gx*q3);
						k13=0.5 * ( gz*q0 + gy*q1 - gx*q2);
						
						k20=0.5 * (halfT*(q0+halfT*k10) + (halfT-gx)*(q1+halfT*k11) + (halfT-gy)*(q2+halfT*k12) + (halfT-gz)*(q3+halfT*k13));
						k21=0.5 * ((halfT+gx)*(q0+halfT*k10) + halfT*(q1+halfT*k11) + (halfT+gz)*(q2+halfT*k12) + (halfT-gy)*(q3+halfT*k13));
						k22=0.5 * ((halfT+gy)*(q0+halfT*k10) + (halfT-gz)*(q1+halfT*k11) + halfT*(q2+halfT*k12) + (halfT+gx)*(q3+halfT*k13));
						k23=0.5 * ((halfT+gz)*(q0+halfT*k10) + (halfT+gy)*(q1+halfT*k11) + (halfT-gx)*(q2+halfT*k12) + halfT*(q3+halfT*k13));
						
						k30=0.5 * (halfT*(q0+halfT*k20) + (halfT-gx)*(q1+halfT*k21) + (halfT-gy)*(q2+halfT*k22) + (halfT-gz)*(q3+halfT*k23));
						k31=0.5 * ((halfT+gx)*(q0+halfT*k20) + halfT*(q1+halfT*k21) + (halfT+gz)*(q2+halfT*k22) + (halfT-gy)*(q3+halfT*k23));
						k32=0.5 * ((halfT+gy)*(q0+halfT*k20) + (halfT-gz)*(q1+halfT*k21) + halfT*(q2+halfT*k22) + (halfT+gx)*(q3+halfT*k23));
						k33=0.5 * ((halfT+gz)*(q0+halfT*k20) + (halfT+gy)*(q1+halfT*k21) + (halfT-gx)*(q2+halfT*k22) + halfT*(q3+halfT*k23));
						
						k40=0.5 * (2*halfT*(q0+2*halfT*k30) + (2*halfT-gx)*(q1+2*halfT*k31) + (2*halfT-gy)*(q2+2*halfT*k32) + (2*halfT-gz)*(q3+2*halfT*k33));
						k41=0.5 * ((2*halfT+gx)*(q0+2*halfT*k30) + 2*halfT*(q1+2*halfT*k31) + (2*halfT+gz)*(q2+2*halfT*k32) + (2*halfT-gy)*(q3+2*halfT*k33));
						k42=0.5 * ((2*halfT+gy)*(q0+2*halfT*k30) + (2*halfT-gz)*(q1+2*halfT*k31) + 2*halfT*(q2+2*halfT*k32) + (2*halfT+gx)*(q3+2*halfT*k33));
						k43=0.5 * ((2*halfT+gz)*(q0+2*halfT*k30) + (2*halfT+gy)*(q1+2*halfT*k31) + (2*halfT-gx)*(q2+2*halfT*k32) + 2*halfT*(q3+2*halfT*k33));	
						
						q0=q0 + 2*halfT/6.0 * (k10+2*k20+2*k30+k40);
						q1=q1 + 2*halfT/6.0 * (k11+2*k21+2*k31+k41);
						q2=q2 + 2*halfT/6.0 * (k12+2*k22+2*k32+k42);
						q3=q3 + 2*halfT/6.0 * (k13+2*k23+2*k33+k43);
						
           // normalise quaternion
           norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
           q0 = q0 * norm;
           q1 = q1 * norm;
           q2 = q2 * norm;
           q3 = q3 * norm;
					 
					 *pitch = -asin(-2 * q1 * q3 + 2 * q0 * q2)* 57.3;	// pitch
					 *roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1)* 57.3;	// roll
					 *yaw   = -atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;	//yaw

}




uint16_t sphereFit(float    d[][3],
                   uint16_t N,
                   uint16_t MaxIterations,
                   float    Err,
                   uint16_t Population[][3],
                   float    SphereOrigin[],
                   float     *SphereRadius)
{
    uint8_t  c;
    uint16_t i, Iterations;
    float    s[3], s2[3], s3[3], sum[3], sum2[3], sum3[3];
    float    x2sum[3], y2sum[3], z2sum[3];
    float    xy_sum, xz_sum, yz_sum;
    float    XY, XZ, YZ, X2Z, Y2X, Y2Z, Z2X, X2Y, Z2Y;
    float    QS, QB, Q0, Q1, Q2;
    float    R2, C[3], C2[3], Delta[3], Denom[3];
    float    F0, F1, F2, F3, F4;
    float    di2[3];
    float    SizeR;

    for (c = XAXIS; c <= ZAXIS; c++)
    {
        s[c] = s2[c] = s3[c] = sum[c] = x2sum[c] = y2sum[c] = z2sum[c] = 0.0f;

        Population[0][c] = Population[1][c] = 0;
    }

    xy_sum = xz_sum = yz_sum = 0.0f;

    for (i = 0; i < N; i++)
    {
        for (c = XAXIS; c <= ZAXIS; c++)
        {
            di2[c] = SQR(d[i][c]);

            s[c]  += d[i][c];
            s2[c] += di2[c];
            s3[c] += di2[c] * d[i][c];

            Population[d[i][c] > 0.0f][c]++;
        }

        xy_sum += d[i][XAXIS] * d[i][YAXIS];
        xz_sum += d[i][XAXIS] * d[i][ZAXIS];
        yz_sum += d[i][YAXIS] * d[i][ZAXIS];

        x2sum[YAXIS] += di2[XAXIS] * d[i][YAXIS];
        x2sum[ZAXIS] += di2[XAXIS] * d[i][ZAXIS];

        y2sum[XAXIS] += di2[YAXIS] * d[i][XAXIS];
        y2sum[ZAXIS] += di2[YAXIS] * d[i][ZAXIS];

        z2sum[XAXIS] += di2[ZAXIS] * d[i][XAXIS];
        z2sum[YAXIS] += di2[ZAXIS] * d[i][YAXIS];
    }

    SizeR = 1.0f / (float) N;

    for (c = XAXIS; c <= ZAXIS; c++)
    {
        sum[c]  = s[c]  * SizeR; //sum( X[n]   )
        sum2[c] = s2[c] * SizeR; //sum( X[n]^2 )
        sum3[c] = s3[c] * SizeR; //sum( X[n]^3 )
    }

    XY = xy_sum * SizeR;         //sum( X[n] * Y[n] )
    XZ = xz_sum * SizeR;         //sum( X[n] * Z[n] )
    YZ = yz_sum * SizeR;         //sum( Y[n] * Z[n] )

    X2Y = x2sum[YAXIS] * SizeR;  //sum( X[n]^2 * Y[n] )
    X2Z = x2sum[ZAXIS] * SizeR;  //sum( X[n]^2 * Z[n] )
    Y2X = y2sum[XAXIS] * SizeR;  //sum( Y[n]^2 * X[n] )
    Y2Z = y2sum[ZAXIS] * SizeR;  //sum( Y[n]^2 * Z[n] )
    Z2X = z2sum[XAXIS] * SizeR;  //sum( Z[n]^2 * X[n] )
    Z2Y = z2sum[YAXIS] * SizeR;  //sum( Z[n]^2 * Y[n] )

    //Reduction of multiplications
    F0 = sum2[XAXIS] + sum2[YAXIS] + sum2[ZAXIS];
    F1 = 0.5f * F0;
    F2 = -8.0f * (sum3[XAXIS] + Y2X + Z2X);
    F3 = -8.0f * (X2Y + sum3[YAXIS] + Z2Y);
    F4 = -8.0f * (X2Z + Y2Z + sum3[ZAXIS]);

    for (c = XAXIS; c <= ZAXIS; c++)
    {
        C[c]  = sum[c];
        C2[c] = SQR(C[c]);
    }

    QS = C2[XAXIS] + C2[YAXIS] + C2[ZAXIS];
    QB = -2.0f * (SQR(C[XAXIS]) + SQR(C[YAXIS]) + SQR(C[ZAXIS]));
    R2 = F0 + QB + QS;
    Q0 = 0.5f * (QS - R2);
    Q1 = F1 + Q0;
    Q2 = 8.0f * (QS - R2 + QB + F0);

    Iterations = 0;

    do
    {
        for (c = XAXIS; c <= ZAXIS; c++)
        {
            Denom[c] = Q2 + 16.0f * (C2[c] - 2.0f * C[c] * sum[c] + sum2[c]);

            if (Denom[c] == 0.0f)
                Denom[c] = 1.0f;
        }

        Delta[XAXIS] = -((F2 + 16.0f * (C[YAXIS] * XY + C[ZAXIS] * XZ + sum[XAXIS] * (-C2[XAXIS] - Q0)
                                        + C[XAXIS] * (sum2[XAXIS] + Q1 - C[ZAXIS] * sum[ZAXIS] - C[YAXIS] * sum[YAXIS]))) / Denom[XAXIS]);

        Delta[YAXIS] = -((F3 + 16.0f * (C[XAXIS] * XY + C[ZAXIS] * YZ + sum[YAXIS] * (-C2[YAXIS] - Q0)
                                        + C[YAXIS] * (sum2[YAXIS] + Q1 - C[XAXIS] * sum[XAXIS] - C[ZAXIS] * sum[ZAXIS]))) / Denom[YAXIS]);

        Delta[ZAXIS] = -((F4 + 16.0f * (C[XAXIS] * XZ + C[YAXIS] * YZ + sum[ZAXIS] * (-C2[ZAXIS] - Q0)
                                        + C[ZAXIS] * (sum2[ZAXIS] + Q1 - C[XAXIS] * sum[XAXIS] - C[YAXIS] * sum[YAXIS]))) / Denom[ZAXIS]);

        for (c = XAXIS; c <= ZAXIS; c++)
        {
            C[c] += Delta[c];
            C2[c] = SQR(C[c]);
        }

        QS = C2[XAXIS] + C2[YAXIS] + C2[ZAXIS];
        QB = -2.0f * (C[XAXIS] * sum[XAXIS] + C[YAXIS] * sum[YAXIS] + C[ZAXIS] * sum[ZAXIS]);
        R2 = F0 + QB + QS;
        Q0 = 0.5f * (QS - R2);
        Q1 = F1 + Q0;
        Q2 = 8.0f * (QS - R2 + QB + F0);

        Iterations++;
    }
    while ((Iterations < 50) || ((Iterations < MaxIterations) && ((SQR(Delta[XAXIS]) + SQR(Delta[YAXIS]) + SQR(Delta[ZAXIS])) > Err)));

    for (c = XAXIS; c <= ZAXIS; c++)
        SphereOrigin[c] = C[c];

    *SphereRadius = sqrt(R2);

    return (Iterations);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
