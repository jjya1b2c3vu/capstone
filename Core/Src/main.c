/* USER CODE BEGIN Header */
/***
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPU9250.h"
#include "stdio.h"
#include "math.h"
#include "stdlib.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t MPU9250 = 0xD0;
uint8_t AK8963 = 0x18;
int16_t Accel_X, Accel_Y, Accel_Z;
int16_t Gyro_X, Gyro_Y, Gyro_Z;
int16_t Mag_X, Mag_Y, Mag_Z;
int16_t Mag_AdX, Mag_AdY, Mag_AdZ;
int8_t ASAX, ASAY, ASAZ;  //sensitivity adjustment values
///모터 ???????????
int flag=0;
float taget_Roll = 0;
float taget_Pitch = -2;
float PID_Roll, PID_Pitch;
#define PI      3.1415926535
#define ANGLE   0.1428572
#define delay_ms     HAL_Delay
#define SYS_CLOCK    72
#define SYSTICK_LOAD 71999
#define millis()     HAL_GetTick()
extern __IO uint32_t uwTick;
float Duty_Rate_Roll[360] = {0.00};
float Duty_Rate_Pitch[360] = {0.00};
int chan1_1 = 0, chan1_2 = 120, chan1_3 = 240;
int chan2_1 = 0, chan2_2 = 120, chan2_3 = 240;

void motor_control(float Roll,float Pitch);
void motor1_Angle(int dir);
void motor2_Angle(int dir);
float get_init_angle();
float FUNC_PID_Roll();
float FUNC_PID_Pitch();
///모터 ???????????  ?
float Ac_X1, Ac_Y1, Ac_Z1, Gy_X1, Gy_Y1, Gy_Z1, Mg_X1, Mg_Y1, Mg_Z1, Mag_X0, Mag_Y0, Mag_Z0;
float Ac_X2, Ac_Y2, Ac_Z2, Bias_Gy_X, Bias_Gy_Y, Bias_Gy_Z, Bias_Ac_X, Bias_Ac_Y, Bias_Ac_Z, Offset_Mag_X, Offset_Mag_Y, Offset_Mag_Z;
float Deg_AX, Deg_AY, Deg_AZ, Deg_GX, Deg_GY, Deg_GZ, Deg_XC, Deg_YC, Deg_ZC, Roll, Pitch, Yaw, mag_scale_X, mag_scale_Y, mag_scale_Z;
float now_Roll=0, now_Pitch=0, Roll_gap,Pitch_gap;// ????????????  ?   측정?
float dt, alpha, beta, Max_X, Max_Y, Max_Z, Min_X, Min_Y, Min_Z;
float Yaw_G, Yaw_M;
float mx, my, mz;
float r_mx,r_my, Xm, Ym;
#define RAD2DEG (180.0 / M_PI) // 57.29
#define DEG2RAD (M_PI / 180) // 0.0174
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
/*---------------------------------------------------------------------------*/
int _write(int32_t file, uint8_t *ptr, int32_t len);
void MPU9250_Write_bits(uint8_t Address, uint8_t bitStart, uint8_t length, uint8_t data);
uint8_t MPU9250_Read(uint8_t Address);
void MPU9250_Write(uint8_t Address, uint8_t data);
void init_MPU9250(void);
void init_AK8963(void);
void read_MPU9250_data(void);
uint8_t AK8963_Read(uint8_t Address);
void AK8963_Write(uint8_t Address, uint8_t data);
void AK8963_Write_bits(uint8_t Address, uint8_t bitStart, uint8_t length, uint8_t data);
void read_AK8963_data(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void MPU9250_Calibration();
void AK8963_Offset();
void Sensor_Calculate();
void MakeVelProfile(float maxVel, float accel);
/*--------------------------------------------------------------------------------*/
//printf
int _write(int32_t file, uint8_t *ptr, int32_t len)
{
  HAL_UART_Transmit(&huart2, ptr, len, 10);
  return len;
}
///
uint8_t MPU9250_Read(uint8_t Address)
{
  uint8_t data;
  HAL_I2C_Mem_Read(&hi2c1, MPU9250, Address, 1, (uint8_t *)&data, 1, 10);
  return data;
}

uint8_t AK8963_Read(uint8_t Address)
{
  uint8_t data;
  HAL_I2C_Mem_Read(&hi2c1, AK8963, Address, 1, (uint8_t *)&data, 1, 10);
  return data;
}

///
void MPU9250_Write(uint8_t Address, uint8_t data)
{
  HAL_I2C_Mem_Write(&hi2c1, MPU9250, Address, 1, (uint8_t *)&data, 1, 10);
}

void AK8963_Write(uint8_t Address, uint8_t data)
{
  HAL_I2C_Mem_Write(&hi2c1, AK8963, Address, 1, (uint8_t *)&data, 1, 10);
}

void init_AK8963(void)
{
   printf("AK8963 I2C Address is 0x%02X(7bit value)    it has to be 0x0C\r\n", AK8963>>1);
   MPU9250_Write_bits(0x37, 1, 1, ENABLE);   ///Mag Enable
   uint8_t temp_AK = AK8963_Read(AK8963_RA_WHO_AM_I);  // AK8963_RA_WHO_AM_I = 0x00
   printf("Who am i = 0x%02X  /// it has to be 0x48\r\n", temp_AK);
   printf("AK8963 Initialize...\r\n");
   printf("----------------------------------------------------------------------------------\r\n");
   //MPU9250_Write_bits(0x37, 1, 1, DISABLE);      ///Mag Disable
   AK8963_Write_bits(0x0B, 0, 1, 0x01);   // CNTL2 0000_0001
   //AK8963_Write_bits(0x0A, 0, 5, 0b10001);   //CNTL1 0001_0010
   HAL_Delay(1000);
}
///
void init_MPU9250(void)
{

   printf("MPU9250 I2C Address is 0x%02X(7bit value)     it has to be 0x68\r\n", MPU9250>>1);
   uint8_t temp_MPU = MPU9250_Read(MPU9250_RA_WHO_AM_I); // MPU9250_RA_WHO_AM_I = 0x75
   printf("Who am i = 0x%02X  /// it has to be 0x71\r\n", temp_MPU);
   printf("MPU9250 Initialize...\r\n");
   printf("----------------------------------------------------------------------------------\r\n");

   HAL_Delay(100);
   ///Power Management 1, sleep disable
   MPU9250_Write_bits(MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_SLEEP_BIT, MPU9250_PWR1_SLEEP_LENGTH, DISABLE);
   HAL_Delay(10);
   ///
   MPU9250_Write_bits(MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_CLKSEL_BIT, MPU9250_PWR1_CLKSEL_LENGTH, MPU9250_CLOCK_INTERNAL);
   ///gyro set 250d/s
   MPU9250_Write_bits(MPU9250_RA_GYRO_CONFIG, MPU9250_GCONFIG_FS_SEL_BIT, MPU9250_GCONFIG_FS_SEL_LENGTH, MPU9250_GYRO_FS_250);
   ///acc set
   MPU9250_Write_bits(MPU9250_RA_ACCEL_CONFIG, MPU9250_ACONFIG_AFS_SEL_BIT, MPU9250_ACONFIG_AFS_SEL_LENGTH, MPU9250_ACCEL_FS_2);
   ///ACC DLPF
   MPU9250_Write_bits(MPU9250_ACCEL_DLPF_CONFIG, MPU9250_ACCEL_DLPF_BIT, MPU9250_ACCEL_DLPF_LENGTH, MPU9250_ACCEL_DLPF_VALUE);
}
///
void MPU9250_Write_bits(uint8_t Address, uint8_t bitStart, uint8_t length, uint8_t data)
{
  uint8_t tmp = 0;
  HAL_I2C_Mem_Read(&hi2c1, MPU9250, Address, 1, (uint8_t *)&tmp, 1, 10);
  uint8_t mask = 0;
  switch(length){
    case 1: mask = 0x01; break;
    case 2: mask = 0x03; break;
    case 3: mask = 0x07; break;
    case 4: mask = 0x0F; break;
    case 5: mask = 0x1F; break;
    case 6: mask = 0x3F; break;
    case 7: mask = 0x7F; break;
    case 8: mask = 0xFF; break;
  }
  tmp &= ~(mask << bitStart);
  tmp |= (data << bitStart);
  HAL_I2C_Mem_Write(&hi2c1, MPU9250, Address, 1, (uint8_t *)&tmp, 1, 10);
}

void AK8963_Write_bits(uint8_t Address, uint8_t bitStart, uint8_t length, uint8_t data)
{
  uint8_t tmp = 0;
  HAL_I2C_Mem_Read(&hi2c1, AK8963, Address, 1, (uint8_t *)&tmp, 1, 10);
  uint8_t mask = 0;
  switch(length){
    case 1: mask = 0x01; break;
    case 2: mask = 0x03; break;
    case 3: mask = 0x07; break;
    case 4: mask = 0x0F; break;
    case 5: mask = 0x1F; break;
    case 6: mask = 0x3F; break;
    case 7: mask = 0x7F; break;
    case 8: mask = 0xFF; break;
  }
  tmp &= ~(mask << bitStart);
  tmp |= (data << bitStart);
  HAL_I2C_Mem_Write(&hi2c1, AK8963, Address, 1, (uint8_t *)&tmp, 1, 10);
}

void read_MPU9250_data(void)
{
  Accel_X = (MPU9250_Read(MPU9250_RA_ACCEL_XOUT_H)<<8) | MPU9250_Read(MPU9250_RA_ACCEL_XOUT_L);
  Accel_Y = (MPU9250_Read(MPU9250_RA_ACCEL_YOUT_H)<<8) | MPU9250_Read(MPU9250_RA_ACCEL_YOUT_L);
  Accel_Z = (MPU9250_Read(MPU9250_RA_ACCEL_ZOUT_H)<<8) | MPU9250_Read(MPU9250_RA_ACCEL_ZOUT_L);
  Gyro_X  = (MPU9250_Read(MPU9250_RA_GYRO_XOUT_H)<<8)  | MPU9250_Read(MPU9250_RA_GYRO_XOUT_L);
  Gyro_Y  = (MPU9250_Read(MPU9250_RA_GYRO_YOUT_H)<<8)  | MPU9250_Read(MPU9250_RA_GYRO_YOUT_L);
  Gyro_Z  = (MPU9250_Read(MPU9250_RA_GYRO_ZOUT_H)<<8)  | MPU9250_Read(MPU9250_RA_GYRO_ZOUT_L);
}

void read_AK8963_data(void)
{
   /*
   Mag_X = (AK8963_Read(AK8963_RA_MAG_XOUT_H)<<8) | AK8963_Read(AK8963_RA_MAG_XOUT_L);
   Mag_Y = (AK8963_Read(AK8963_RA_MAG_YOUT_H)<<8) | AK8963_Read(AK8963_RA_MAG_YOUT_L);
   Mag_Z = (AK8963_Read(AK8963_RA_MAG_ZOUT_H)<<8) | AK8963_Read(AK8963_RA_MAG_ZOUT_L);
   */
   Mag_X = (AK8963_Read(AK8963_RA_MAG_YOUT_H)<<8) | AK8963_Read(AK8963_RA_MAG_YOUT_L);
   Mag_Y = (AK8963_Read(AK8963_RA_MAG_XOUT_H)<<8) | AK8963_Read(AK8963_RA_MAG_XOUT_L);
   Mag_Z = -((AK8963_Read(AK8963_RA_MAG_ZOUT_H)<<8) | AK8963_Read(AK8963_RA_MAG_ZOUT_L));
   ASAX = (AK8963_Read(0x10));
   ASAY = (AK8963_Read(0x11));
   ASAZ = (AK8963_Read(0x12));
}

void MPU9250_Calibration()
{
   printf("MPU9250_Bias\r\n");
   int loop = 20;
   for(int i = 0 ; i < loop ; i++)
   {
      read_MPU9250_data();
      Bias_Gy_X += Gyro_X;
      Bias_Gy_Y += Gyro_Y;
      Bias_Gy_Z += Gyro_Z;
   }

   Bias_Gy_X /= loop;
   Bias_Gy_Y /= loop;
   Bias_Gy_Z /= loop;
   printf("Bias_Gy_X: %5.2f,  Bias_Gy_Y: %5.2f,  Bias_Gy_Z: %5.2f \r\n", Bias_Gy_X, Bias_Gy_Y, Bias_Gy_Z);
   //printf("----------------------------------------------------------------------------------\r\n");
}

void AK8963_Offset()
{
   printf("AK8963_Offset\r\n");
   int loop = 200;
   int16_t temp_max[3]={0,0,0};
   int16_t temp_min[3]={10000,10000,10000};
   for(int i = 0 ; i < loop ; i++)
   {
      AK8963_Write_bits(0x0A, 0, 5, 0b10001);   //CNTL1 0001_0010
      HAL_Delay(50);
      read_AK8963_data();
      Max_X=MAX(Mag_X, temp_max[0]);
      Max_Y=MAX(Mag_Y, temp_max[1]);
      Max_Z=MAX(Mag_Z, temp_max[2]);
      Min_X=MIN(Mag_X, temp_min[0]);
      Min_Y=MIN(Mag_Y, temp_min[1]);
      Min_Z=MIN(Mag_Z, temp_min[2]);
      temp_max[0] = Max_X; temp_max[1] = Max_Y; temp_max[2] = Max_Z;
      temp_min[0] = Min_X; temp_min[1] = Min_Y; temp_min[2] = Min_Z;
   }

   Offset_Mag_X = (temp_max[0] + temp_min[0])/2;
   Offset_Mag_Y = (temp_max[1] + temp_min[1])/2;
   Offset_Mag_Z = (temp_max[2] + temp_min[2])/2;

   /* Offset Check Printf */
   printf("Max_Mag_X: %6d,  min_Mag_X: %6d\r\n", temp_max[0], temp_min[0]);
   printf("Max_Mag_Y: %6d,  min_Mag_Y: %6d\r\n", temp_max[1], temp_min[1]);
   printf("Max_Mag_Z: %6d,  min_Mag_Z: %6d\r\n", temp_max[2], temp_min[2]);
   printf("Offset_Mag_X: %5.2f,  Offset_Mag_Y: %5.2f,  Offset_Mag_Z: %5.2f \r\n", Offset_Mag_X, Offset_Mag_Y, Offset_Mag_Z);

   printf("---------------------------------------------------------------------------------- \r\n");
}
float get_init_angle()
{
   float temp_array[100];
   float taget_Roll=0;
   for (int i=0;i<100;i++)
   {
      temp_array[i]=Roll;
   }
   for(int j=0;j<100;j++)
   {
      taget_Roll+=temp_array[j];
   }
   return taget_Roll;
}
#define BIN2GYR 131.072
#define BIN2MAG 0.15
#define BIN2ACC 16384.0
void Sensor_Calculate()
{
   //Bias_Gy_X = -83.3693; Bias_Gy_Y = 221.402; Bias_Gy_Z = -180.1906;
   Bias_Gy_X = 0.0; Bias_Gy_Y = 0.0; Bias_Gy_Z = 0.0;
   Offset_Mag_X = 250;   Offset_Mag_Y = 85; Offset_Mag_Z = 320;
   mag_scale_X = 334.6 * BIN2MAG; mag_scale_Y = 270.0 * BIN2MAG; mag_scale_Z = 340.0 * BIN2MAG;
   ASAX = 175, ASAY = 176, ASAZ = 92;
   Mag_AdX = 1.18, Mag_AdY = 1.19, Mag_AdX = 0.86;
   dt = 0.01, alpha = 0.92, beta =0.94;

   /* correction Mag_Raw_Data */
   Mag_X0 = ((float)(Mag_X * BIN2MAG * Mag_AdX)) - ((float)(Offset_Mag_X * BIN2MAG * Mag_AdX));
   Mag_Y0 = ((float)(Mag_Y * BIN2MAG * Mag_AdY)) - ((float)(Offset_Mag_Y * BIN2MAG * Mag_AdY));
   Mag_Z0 = ((float)(Mag_Z * BIN2MAG * Mag_AdZ)) - ((float)(Offset_Mag_Z * BIN2MAG * Mag_AdZ));

   Mg_X1 = Mag_X0 / mag_scale_X;
   Mg_Y1 = Mag_Y0 / mag_scale_Y;
   Mg_Z1 = Mag_Z0 / mag_scale_Z;

   /*  Acc: g, Gyr: °/s, Mag: µT  */
   Ac_X1 = ((float)Accel_X) / BIN2ACC;
   Ac_Y1 = ((float)Accel_Y) / BIN2ACC;
   Ac_Z1 = ((float)Accel_Z) / BIN2ACC;
   Gy_X1 = ((float)Gyro_X - Bias_Gy_X) / BIN2GYR;
   Gy_Y1 = ((float)Gyro_Y - Bias_Gy_Y) / BIN2GYR;
   Gy_Z1 = ((float)Gyro_Z - Bias_Gy_Z) / BIN2GYR;


   /* Acc Angle */
   Deg_AX = atan(Ac_Y1 / sqrt(pow(Ac_X1, 2) + pow(Ac_Z1, 2))) * RAD2DEG;
   Deg_AY = atan(-1 * Ac_X1 / sqrt(pow(Ac_Y1, 2) + pow(Ac_Z1, 2))) * RAD2DEG;
   Deg_AZ = atan(sqrt(pow(Ac_X1, 2) + pow(Ac_Y1, 2)) / Ac_Z1) * RAD2DEG; // no meaning

   /* Gyro Angle */
   Deg_GX = Roll + Gy_X1 * dt;
   Deg_GY = Pitch + Gy_Y1 * dt;
   Deg_GZ = Yaw + Gy_Z1 * dt;

   /* Roll, Pitch, Yaw Complementary filter */
   Xm =  (-Mg_Y1 * cos(Roll * DEG2RAD) + Mg_Z1 * sin(Roll * DEG2RAD));
   Ym =  (Mg_X1 * cos(Pitch * DEG2RAD) + Mg_Y1 * sin(Pitch * DEG2RAD) * sin(Roll * DEG2RAD) + Mg_Z1 * sin(Pitch * DEG2RAD) * cos(Roll * DEG2RAD));
   Yaw_M = RAD2DEG * atan2(Xm, Ym);  //atan2(분자, 분모)
   Yaw_G = Deg_GZ;

   Roll = alpha * Deg_GX + (1 - alpha) * Deg_AX;
   Pitch = alpha * Deg_GY + (1 - alpha) * Deg_AY;
   Yaw = beta * Yaw_G + (1 - beta) * Yaw_M;
}
////motor start
uint32_t micros() {
  return (uwTick&0x3FFFFF)*1000 + (SYSTICK_LOAD-SysTick->VAL)/SYS_CLOCK;
}

void HAL_Delay(__IO uint32_t Delay) {
  uint32_t tickstart = HAL_GetTick();

  while((millis() - tickstart) < Delay);
}

void delay_us(uint32_t us) {
  uint32_t temp = micros();
  uint32_t comp = temp + us;
  uint8_t  flag = 0;
  while(comp > temp){
    if(((uwTick&0x3FFFFF)==0)&&(flag==0)){
      flag = 1;
    }
    if(flag) temp = micros() + 0x400000UL * 1000;
    else     temp = micros();
  }
}

#define num1    1
#define num2   1
#define Time    100
void motor_control(float roll, float pitch)
{
    unsigned int step_Roll  = fabs( roll  / (num1 * ANGLE) ) + 0.5;
    unsigned int step_Pitch = fabs( pitch / (num2 * ANGLE) ) + 0.5;
    unsigned int cnt_Roll = 0, cnt_Pitch = 0;
    int dir_Roll = 0, dir_Pitch = 0;
    int max_step = 0;

    if(roll >= 0) dir_Roll = 1;    else dir_Roll = 0;
    if(pitch >= 0) dir_Pitch = 1;    else dir_Pitch = 0;
    if(step_Roll >= step_Pitch) max_step = step_Roll;    else max_step = step_Pitch;

    for(int i = 0; i < max_step; i++)
    {
       if(step_Pitch > cnt_Pitch)
       {
          motor2_Angle(dir_Pitch);
          cnt_Pitch++;
       }

       if(step_Roll > cnt_Roll)
       {
          motor1_Angle(dir_Roll);
          cnt_Roll++;
       }
    }
    // delay_us(Time);

}

void motor1_Angle(int dir)                                      // motor angle control
{
     if(dir == 1)                                               // ?  방향
     {
         htim1.Instance -> CCR1 = Duty_Rate_Roll[chan1_1];
         htim1.Instance -> CCR2 = Duty_Rate_Roll[chan1_2];
         htim1.Instance -> CCR3 = Duty_Rate_Roll[chan1_3];

         chan1_1 = chan1_1 + num1;
         chan1_2 = chan1_2 + num1;
         chan1_3 = chan1_3 + num1;

         if(chan1_1 >= 360 ) {chan1_1 = 0;}
         if(chan1_2 >= 360 ) {chan1_2 = 0;}
         if(chan1_3 >= 360 ) {chan1_3 = 0;}
      }

      else if(dir == 0)                                          // ?  방향
      {
         htim1.Instance -> CCR1 = Duty_Rate_Roll[chan1_1];
         htim1.Instance -> CCR2 = Duty_Rate_Roll[chan1_2];
         htim1.Instance -> CCR3 = Duty_Rate_Roll[chan1_3];

         chan1_1 = chan1_1 - num1;
         chan1_2 = chan1_2 - num1;
         chan1_3 = chan1_3 - num1;

         if(chan1_1 < 0 ) {chan1_1 = 359;}
         if(chan1_2 < 0 ) {chan1_2 = 359;}
         if(chan1_3 < 0 ) {chan1_3 = 359;}
      }
}

void motor2_Angle(int dir)                                      // motor angle control
{
      if(dir == 1)
      {
         htim2.Instance -> CCR1 = Duty_Rate_Pitch[chan2_1];
         htim2.Instance -> CCR2 = Duty_Rate_Pitch[chan2_2];
         htim2.Instance -> CCR3 = Duty_Rate_Pitch[chan2_3];

         chan2_1 = chan2_1 + num2;
         chan2_2 = chan2_2 + num2;
         chan2_3 = chan2_3 + num2;

         if(chan2_1 >= 360 ) {chan2_1 = 0;}
         if(chan2_2 >= 360 ) {chan2_2 = 0;}
         if(chan2_3 >= 360 ) {chan2_3 = 0;}
      }

      else if(dir == 0)
      {
         htim2.Instance -> CCR1 = Duty_Rate_Pitch[chan2_1];
         htim2.Instance -> CCR2 = Duty_Rate_Pitch[chan2_2];
         htim2.Instance -> CCR3 = Duty_Rate_Pitch[chan2_3];

         chan2_1 = chan2_1 - num2;
         chan2_2 = chan2_2 - num2;
         chan2_3 = chan2_3 - num2;

         if(chan2_1 < 0 ) {chan2_1 = 359;}
         if(chan2_2 < 0 ) {chan2_2 = 359;}
         if(chan2_3 < 0 ) {chan2_3 = 359;}
      }
}



float PrevError_C = 0, PrevError_C2 = 0;
float Error_Sum = 0, Error_Sum2 = 0;
float dt = 0.01;
float Output = 0, Output2 = 0, Error = 0, Error2 = 0;
float P_term = 0, I_term = 0, D_term = 0;
float P_term2 = 0, I_term2 = 0, D_term2 = 0;

#define Kp_r   0.30
#define Ki_r   0.00
#define Kd_r   0.005
float FUNC_PID_Roll()
{
   Error = taget_Roll - Roll;  Error_Sum += Error * dt;

   P_term = Kp_r * Error;
   I_term = Ki_r * Error_Sum;
   D_term = Kd_r * ((Error - PrevError_C)/dt);

   Output = P_term + I_term + D_term;
   PrevError_C = Error;

   return (Output);
}


#define Kp_p     0.2   //0.30
#define Ki_p   0.00
#define Kd_p    0.02  //0.045
float FUNC_PID_Pitch() //negative feedback system
{
   Error2 = -(taget_Pitch - Pitch);  Error_Sum2 += Error2 * dt;
   //Error2 = - Pitch;  Error_Sum2 += Error2 * dt;

   P_term2 = Kp_p * Error2;
   I_term2 = Ki_p * Error_Sum2;
   D_term2 = Kd_p * ((Error2 - PrevError_C2) / dt);

   Output2 = P_term2 + I_term2 + D_term2;
   PrevError_C2 = Error2;

   return (Output2);
}

////motor end
//ISP_func


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) ///new
{
   if (htim->Instance == TIM4)
   {
      read_MPU9250_data();
      read_AK8963_data();
      Sensor_Calculate();
      flag += 1;
   }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

   for(int i = 0; i < 360; i++)
   {
      Duty_Rate_Roll[i] = 180 + 180 * sin((2*PI*i)/360);  //50 +- 50 %
      Duty_Rate_Pitch[i] = 180 + 180 * sin((2*PI*i)/360);  //50 +- 50 %
   }

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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);          // motor1
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);          // motor2
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

  init_MPU9250();
  init_AK8963();
  //MPU9250_Calibration();
  //AK8963_Offset();
  HAL_TIM_Base_Start_IT(&htim4);///Timer On
  //void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim); //Timer Function --> don't hal_delay
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
     /* Print for Roll Pitch Yaw */
     //printf("Roll: %5.2f   /  Pitch: %5.2f   /  Yaw: %5.2f \r\n", Roll, Pitch, Yaw);
     printf("%5.2f / %5.2f / %5.2f \r\n", Roll, Pitch, Yaw); //for HyperTerminal
     //printf("YPR,%5.2f,%5.2f,%5.2f\r\n", Yaw, Pitch, Roll); //for Processing
     //printf("------------------------------------------------------------\r\n");

     if (flag > 50)
     {
        /* Print for Roll Pitch Yaw */
        //printf("Roll: %5.2f   /  Pitch: %5.2f   /  Yaw: %5.2f \r\n", Roll, Pitch, Yaw);
        //printf("%5.2f/%5.2f/%5.2f \r\n", Roll, Pitch, Yaw); //for HyperTerminal
        //printf("YPR,%5.2f,%5.2f,%5.2f\r\n", Yaw, Pitch, Roll); //for Processing
        //printf("------------------------------------------------------------\r\n");
        flag = 0;
     }



    /* USER CODE END WHILE */

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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
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
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 208;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_ENABLE;
  hi2c1.Init.OwnAddress2 = 24;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_ENABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 10-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 360-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 10-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 360-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 20-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 360-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7200-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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