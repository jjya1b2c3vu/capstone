/*
 * MPU9250.h
 *
 *  Created on: Apr 14, 2020
 *      Author: JUN0
 */


/*
 device address
 MPU9250 = 0x68(7 bit)
 AK8963 = 0x0C(7 bit)
*/

/*
 device who_am_i
 MPU9250 = 0x75 -->  0x71
 AK8963 = 0x00 -->  0x48
*/

#ifndef INC_MPU9250_H_
#define INC_MPU9250_H_
#define MPU9250_ACCEL_DLPF_CONFIG       0x1D
#define MPU9250_ACCEL_DLPF_BIT          0
#define MPU9250_ACCEL_DLPF_LENGTH       3
#define MPU9250_ACCEL_DLPF_VALUE        0b110
#define MPU9250_CLOCK_INTERNAL			0x00
#define MPU9250_ADDRESS					(0x68 << 1)
#define MPU9250_RA_WHO_AM_I				0x75   // 0x71 return
#define MPU9250_RA_PWR_MGMT_1			0x6B
#define MPU9250_RA_PWR_MGMT_2			0x6C
#define MPU9250_INT_PIN_CFG				0x37
#define MPU9250_INT_PIN_CFG_BIT			7
#define MPU9250_INT_PIN_CFG_LENGTH		8
#define MPU9250_INT_PIN_CFG_SET			0x22

#define MPU9250_RA_GYRO_CONFIG			0x1B
#define MPU9250_GCONFIG_FS_SEL_BIT      3
#define MPU9250_GCONFIG_FS_SEL_LENGTH   2
#define MPU9250_GYRO_FS_250         	0x00
#define MPU9250_GYRO_FS_500         	0x01
#define MPU9250_GYRO_FS_1000        	0x02
#define MPU9250_GYRO_FS_2000        	0x03

#define MPU9250_RA_ACCEL_CONFIG			0x1C
#define MPU9250_ACONFIG_AFS_SEL_BIT		3
#define MPU9250_ACONFIG_AFS_SEL_LENGTH	2
#define MPU9250_ACCEL_FS_2          	0x00
#define MPU9250_ACCEL_FS_4          	0x01
#define MPU9250_ACCEL_FS_8          	0x02
#define MPU9250_ACCEL_FS_16         	0x03

#define MPU9250_RA_ACCEL_XOUT_H			0x3B
#define MPU9250_RA_ACCEL_XOUT_L			0x3C
#define MPU9250_RA_ACCEL_YOUT_H			0x3D
#define MPU9250_RA_ACCEL_YOUT_L			0x3E
#define MPU9250_RA_ACCEL_ZOUT_H			0x3F
#define MPU9250_RA_ACCEL_ZOUT_L			0x40

#define MPU9250_RA_GYRO_XOUT_H      	0x43
#define MPU9250_RA_GYRO_XOUT_L      	0x44
#define MPU9250_RA_GYRO_YOUT_H      	0x45
#define MPU9250_RA_GYRO_YOUT_L      	0x46
#define MPU9250_RA_GYRO_ZOUT_H      	0x47
#define MPU9250_RA_GYRO_ZOUT_L      	0x48

#define MPU9250_PWR1_DEVICE_RESET_BIT		7
#define MPU9250_PWR1_SLEEP_BIT				6
#define MPU9250_PWR1_CYCLE_BIT				5
#define MPU9250_PWR1_SLEEP_LENGTH			1
#define MPU9250_PWR1_CLKSEL_BIT				0
#define MPU9250_PWR1_CLKSEL_LENGTH			3




#define AK8963_ADDRESS					0x0C
#define AK8963_RA_WHO_AM_I				0x00   //0x48 return
#define AK8963_RA_ST1					0x02
#define AK8963_RA_HXL					0x03
#define AK8963_RA_CNTL1					0x0A
#define AK8963_RA_ST2					0x09
#define AK8963_RA_ASAX					0x10

#define AK8963_RA_MAG_XOUT_L      		0x03
#define AK8963_RA_MAG_XOUT_H      		0x04
#define AK8963_RA_MAG_YOUT_L      		0x05
#define AK8963_RA_MAG_YOUT_H      		0x06
#define AK8963_RA_MAG_ZOUT_L     	 	0x07
#define AK8963_RA_MAG_ZOUT_H      		0x08



#endif /* INC_MPU9250_H_ */
