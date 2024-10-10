/*
 * 9250_sensor.c
 *
 *  Created on: Oct 7, 2024
 *      Author: PC_3884
 */

#include "main.h"
#include "stdio.h"
#include "MPU9250.h"

extern I2C_HandleTypeDef hi2c5;

float aRes, gRes, mRes;      																			// her bir LSB iÃ§in Ã¶lÃ§ek Ã§Ã¶zÃ¼nÃ¼rlÃ¼kleri
float ax, ay, az, gx, gy, gz, mx, my, mz;																// sensÃ¶rÃ¼n Ã¶lÃ§eklendirmeden sonraki son halini tutan deÄŸiÅŸkenler
const uint16_t i2c_timeout = 100;

int16_t Accel[3];
int16_t Gyro[3];
int16_t Magneto[3];
float   SelfTest[6];

uint8_t Gscale = GFS_2000DPS;
uint8_t Ascale = AFS_16G;
uint8_t Mscale = MFS_14BITS;
uint8_t Mmode = 0x06;


uint8_t MPU9255_Init(I2C_HandleTypeDef *hi2c5){

	//pre-def. vars
	uint8_t readData;
	uint8_t writeData;
	uint8_t ak8963WhoAmI;

	//9250 who am i okuma
	HAL_I2C_Mem_Read(hi2c5, MPU9250_ADDRESS, WHO_AM_I_MPU9250, 1, &readData, 1, i2c_timeout);
	if (readData == 112) {

	//gyro ve accel baÅŸlatma
	initMPU9250(hi2c5);

	//magneto bypass
	writeData = 0x22;
	HAL_I2C_Mem_Write(hi2c5, MPU9250_ADDRESS, INT_PIN_CFG, 1, &writeData, 1, i2c_timeout);

	//ak8963 who am i okuma
	HAL_I2C_Mem_Read(hi2c5, AK8963_ADDRESS, AK8963_WHO_AM_I, 1, &ak8963WhoAmI, 1, i2c_timeout);// Read WHO_AM_I register for AK8963
	if(ak8963WhoAmI == 0){
		HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_1);
		HAL_Delay(100);
	}
	initAK8963(hi2c5);
return 0;
	}
return 1;
}

void calibrateMPU9250(I2C_HandleTypeDef *hi2c5, float * dest1, float * dest2){
  uint8_t writeData;

	uint8_t calibData[12]; // data array to hold accelerometer and gyro x, y, z, data
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

	//reset
	writeData = 0x80;
	HAL_I2C_Mem_Write(hi2c5, MPU9250_ADDRESS, PWR_MGMT_1, 1, &writeData, 1, i2c_timeout);// Write a one to bit 7 reset bit; toggle reset device
	HAL_Delay(100);

	//saat kaynaÄŸÄ±nÄ±n seÃ§ilmesi, pll jiroskop referansÄ± kullanÄ±ldÄ±
	writeData = 0x01;
	HAL_I2C_Mem_Write(hi2c5, MPU9250_ADDRESS, PWR_MGMT_1, 1, &writeData, 1, i2c_timeout);
	writeData = 0x00;
	HAL_I2C_Mem_Write(hi2c5, MPU9250_ADDRESS, PWR_MGMT_2, 1, &writeData, 1, i2c_timeout);
	HAL_Delay(200);

	// bias hesaplamalarÄ± iÃ§in cihazÄ±n konfigÃ¼rasyonu
	writeData = 0x00;
	HAL_I2C_Mem_Write(hi2c5, MPU9250_ADDRESS, INT_ENABLE, 1, &writeData, 1, i2c_timeout);// Disable all interrupts
	writeData = 0x00;
	HAL_I2C_Mem_Write(hi2c5, MPU9250_ADDRESS, FIFO_EN, 1, &writeData, 1, i2c_timeout);// Disable FIFO
	writeData = 0x00;
	HAL_I2C_Mem_Write(hi2c5, MPU9250_ADDRESS, PWR_MGMT_1, 1, &writeData, 1, i2c_timeout);// Turn on internal clock source
	writeData = 0x00;
	HAL_I2C_Mem_Write(hi2c5, MPU9250_ADDRESS, I2C_MST_CTRL, 1, &writeData, 1, i2c_timeout);// Disable I2C master
	writeData = 0x00;
	HAL_I2C_Mem_Write(hi2c5, MPU9250_ADDRESS, USER_CTRL, 1, &writeData, 1, i2c_timeout);// Disable FIFO and I2C master modes
	writeData = 0x0C;
	HAL_I2C_Mem_Write(hi2c5, MPU9250_ADDRESS, USER_CTRL, 1, &writeData, 1, i2c_timeout);// Reset FIFO and DMP
	HAL_Delay(15);

	// jiroskop ve ivmeÃ¶lÃ§er iÃ§in bias
	writeData = 0x01;
	HAL_I2C_Mem_Write(hi2c5, MPU9250_ADDRESS, CONFIG, 1, &writeData, 1, i2c_timeout);// Set low-pass filter to 188 Hz
	writeData = 0x00;
	HAL_I2C_Mem_Write(hi2c5, MPU9250_ADDRESS, SMPLRT_DIV, 1, &writeData, 1, i2c_timeout);// Set sample rate to 1 kHz
	writeData = 0x00;
	HAL_I2C_Mem_Write(hi2c5, MPU9250_ADDRESS, GYRO_CONFIG, 1, &writeData, 1, i2c_timeout);// Set gyro full-scale to 250 degrees per second, maximum sensitivity
	writeData = 0x00;
	HAL_I2C_Mem_Write(hi2c5, MPU9250_ADDRESS, ACCEL_CONFIG, 1, &writeData, 1, i2c_timeout);// Set accelerometer full-scale to 2 g, maximum sensitivity

	uint16_t  gyrosensitivity  = 131;
	uint16_t  accelsensitivity = 16384;

	// FIFO konfigÃ¼rasyonu
	writeData = 0x40;
	HAL_I2C_Mem_Write(hi2c5, MPU9250_ADDRESS, USER_CTRL, 1, &writeData, 1, i2c_timeout);
	writeData = 0x78;
	HAL_I2C_Mem_Write(hi2c5, MPU9250_ADDRESS, FIFO_EN, 1, &writeData, 1, i2c_timeout);// Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
	HAL_Delay(40);

	// vei toplama sonunda FIFO kapatma
	writeData = 0x00;
	HAL_I2C_Mem_Write(hi2c5, MPU9250_ADDRESS, FIFO_EN, 1, &writeData, 1, i2c_timeout);// Disable gyro and accelerometer sensors for FIFO
	HAL_I2C_Mem_Read(hi2c5, MPU9250_ADDRESS, FIFO_COUNTH, 1, &calibData[0], 2, i2c_timeout);// read FIFO sample count
	fifo_count = ((uint16_t)calibData[0] << 8) | calibData[1];
	packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

}

void initMPU9250(I2C_HandleTypeDef *hi2c5){

	uint8_t readData;
	uint8_t writeData;

	//cihazÄ± uyandÄ±rma
	writeData = 0x00;
	HAL_I2C_Mem_Write(hi2c5, MPU9250_ADDRESS, PWR_MGMT_1, 1, &writeData, 1, i2c_timeout);

	writeData = 0x01;
	HAL_I2C_Mem_Write(hi2c5, MPU9250_ADDRESS, PWR_MGMT_1, 1, &writeData, 1, i2c_timeout);

	writeData = 0x03;
	HAL_I2C_Mem_Write(hi2c5, MPU9250_ADDRESS, CONFIG, 1, &writeData, 1, i2c_timeout);

	writeData = 0x04;
	HAL_I2C_Mem_Write(hi2c5, MPU9250_ADDRESS, SMPLRT_DIV, 1, &writeData, 1, i2c_timeout);

	HAL_I2C_Mem_Read(hi2c5, MPU9250_ADDRESS, GYRO_CONFIG, 1, &readData, 1, i2c_timeout);
	readData = readData & ~0x03;		 																				// Fchoice bitlerini temizler [1:0]
	readData = readData & ~0x18; 																						//GFS bitlerini temizler[4:3]
	readData = readData | Gscale << 3; 																					// Gyro iÃ§in full scale rangeleri set eder

	writeData = readData;
	HAL_I2C_Mem_Write(hi2c5, MPU9250_ADDRESS, GYRO_CONFIG, 1, &writeData, 1, i2c_timeout);


	HAL_I2C_Mem_Read(hi2c5, MPU9250_ADDRESS, ACCEL_CONFIG, 1, &readData, 1, i2c_timeout);
	readData = readData & ~0x18;       																					// Accel full scale bitlerini temizler [4:3]
	readData = readData | Ascale << 3; 																					// Accel iÃ§in full scale range set eder

	writeData = readData;
	HAL_I2C_Mem_Write(hi2c5, MPU9250_ADDRESS, ACCEL_CONFIG, 1, &writeData, 1, i2c_timeout);

	HAL_I2C_Mem_Read(hi2c5, MPU9250_ADDRESS, ACCEL_CONFIG2, 1, &readData, 1, i2c_timeout);
	readData = readData & ~0x0F; 																						// Accel_fchoice_b (bit 3) ve A_DLPFG (bits [2:0]) temizler
	readData = readData | 0x03;  																						// Accelorometer hÄ±zÄ±nÄ± 1 kHz'e ve bandwidth'i 41 Hz'e set eder

	writeData = readData;
	HAL_I2C_Mem_Write(hi2c5, MPU9250_ADDRESS, ACCEL_CONFIG2, 1, &writeData, 1, i2c_timeout);

}

float accelXBuffer[WINDOW_SIZE] = {0};
float accelYBuffer[WINDOW_SIZE] = {0};
float accelZBuffer[WINDOW_SIZE] = {0};

int accelXIndex = 0;
int accelYIndex = 0;
int accelZIndex = 0;

int accelXCount = 0;
int accelYCount = 0;
int accelZCount = 0;

//applymoving filter fonksiyonunda aslÄ±nda a yerinde index vardÄ±!!!
float applyMovingAverageFilter(float newValue, float* dataBuffer, int* a, int* count) {
    dataBuffer[*a] = newValue;
    *a = (*a + 1) % WINDOW_SIZE;
    if (*count < WINDOW_SIZE) (*count)++;

    float sum = 0.0f;
    for (int i = 0; i < *count; i++) {
        sum += dataBuffer[i];
    }
    return sum / *count;
}

void initAK8963(I2C_HandleTypeDef *hi2c5){

	  uint8_t writeData;

	  //manyetometre uyandÄ±rma
	  writeData = 0x01;
	  HAL_I2C_Mem_Write(hi2c5, AK8963_ADDRESS, AK8963_CNTL2, 1, &writeData, 1, i2c_timeout);

	  //manyetometre power down
	  writeData = 0x00;
	  HAL_I2C_Mem_Write(hi2c5, AK8963_ADDRESS, AK8963_CNTL, 1, &writeData, 1, i2c_timeout);
	  HAL_Delay(100);

	  writeData = Mscale << 4 | Mmode;
	  HAL_I2C_Mem_Write(hi2c5, AK8963_ADDRESS, AK8963_CNTL, 1, &writeData, 1, i2c_timeout);

	  //fuse rom modu
	  writeData = 0x0F;
	  HAL_I2C_Mem_Write(hi2c5, AK8963_ADDRESS, AK8963_CNTL, 1, &writeData, 1, i2c_timeout);// Enter Fuse ROM access mode
	  HAL_Delay(100);


}

void MPU9250_Read_Raw_acc(mpudata_t * mpudata_st) {
    uint8_t Rec_Data[6];

    // Raw data okuma
    HAL_I2C_Mem_Read(&hi2c5, (AD0_LOW << 1) + 1, REG_DATA , 1, Rec_Data, 6, 100);

    Accel[0] = ((int16_t)(Rec_Data[0] << 8) + Rec_Data[1]);
    Accel[1] = ((int16_t)(Rec_Data[2] << 8) + Rec_Data[3]);
    Accel[2] = ((int16_t)(Rec_Data[4] << 8) + Rec_Data[5]);

    // Ã–lÃ§eklenmiÅŸ verileri hesaplama
    getAres();
    ax = (float)Accel[0] * aRes;
    ay = (float)Accel[1] * aRes;
    az = (float)Accel[2] * aRes;

    // Hareketli ortalama filtresi uygulama
    float smoothAccelX = applyMovingAverageFilter(ax, accelXBuffer, &accelXIndex, &accelXCount);
    float smoothAccelY = applyMovingAverageFilter(ay, accelYBuffer, &accelYIndex, &accelYCount);
    float smoothAccelZ = applyMovingAverageFilter(az, accelZBuffer, &accelZIndex, &accelZCount);

    // FiltrelenmiÅŸ verileri yapÄ±landÄ±rmaya atama
    mpudata_st->Accel_X_RAW = smoothAccelX;
    mpudata_st->Accel_Y_RAW = smoothAccelY;
    mpudata_st->Accel_Z_RAW = smoothAccelZ;

    HAL_Delay(3);
}
void MPU9250_Read_Raw_gyro(mpudatag_t * mpudatag_st)
{
	uint8_t Rec_Data[6];

	HAL_I2C_Mem_Read(&hi2c5, (AD0_LOW << 1) + 1, REGg_DATA , 1, Rec_Data, 6, 100);

	Gyro[0] = ((int16_t)(Rec_Data[0] << 8) + Rec_Data [1]);
	Gyro[1] = ((int16_t)(Rec_Data[2] << 8) + Rec_Data [3]);
	Gyro[2] = ((int16_t)(Rec_Data[4] << 8) + Rec_Data [5]);

	getGres();

	gx = (float)Gyro[0]*gRes;
	gy = (float)Gyro[1]*gRes;
	gz = (float)Gyro[2]*gRes;

	mpudatag_st->Gyro_X_RAW = gx;
	mpudatag_st->Gyro_Y_RAW = gy;
	mpudatag_st->Gyro_Z_RAW = gz;

	HAL_Delay(3);
}

void MPU9250_Read_Raw_mag(mpudatam_t * mpudatam_st){

	uint8_t Rec_Data[6];

	HAL_I2C_Mem_Read(&hi2c5, AK8963_ADDRESS, AK8963_XOUT_L, 1, Rec_Data, 6, i2c_timeout);

	Magneto[0] = ((int16_t)Rec_Data[1] << 8) | Rec_Data[0] ;
	Magneto[1] = ((int16_t)Rec_Data[3] << 8) | Rec_Data[2] ;
	Magneto[2] = ((int16_t)Rec_Data[5] << 8) | Rec_Data[4] ;

	getMres();

	mx = (float)Magneto[0]*mRes;
	my = (float)Magneto[1]*mRes;
	mz = (float)Magneto[2]*mRes;

	mpudatam_st->Magneto_X_RAW = mx;
	mpudatam_st->Magneto_Y_RAW = my;
	mpudatam_st->Magneto_Z_RAW = mz;

	HAL_Delay(3);

}

void readTemperature(temp_t * temp_st) {
	      uint8_t temp_h, temp_l;
	      float temp_raw;
	      float temp_o;

	      // SÄ±caklÄ±k yÃ¼ksek ve dÃ¼ÅŸÃ¼k byte'Ä± oku
	      HAL_I2C_Mem_Read(&hi2c5, MPU9250_ADDRESS, TEMP_OUT_H, 1, &temp_h, 1, 100);
	      HAL_I2C_Mem_Read(&hi2c5, MPU9250_ADDRESS, TEMP_OUT_L, 1, &temp_l, 1, 100);

	      float RoomTemp_Offset =0; // Kalibrasyon iÃ§in ofset deÄŸeri
	      float Temp_Sensitivity = 340.0; // Datasheet'ten alÄ±nan sÄ±caklÄ±k hassasiyeti

	      temp_st->temp_raw =  temp_raw = (temp_h << 8) | temp_l;
	      temp_st->temp_o = ((temp_raw - RoomTemp_Offset) / Temp_Sensitivity) + 21.0;
}

void sendSensorData(mpudata_t *mpudata, mpudatag_t *mpudatag)
{

    uint8_t buffer[50];
    int len = snprintf((char*)buffer, sizeof(buffer),

                       "Accel: X=%d, Y=%d, Z=%d | Gyro: X=%d, Y=%d, Z=%d\r\n",
                       mpudata->Accel_X_RAW, mpudata->Accel_Y_RAW, mpudata->Accel_Z_RAW,
                       mpudatag->Gyro_X_RAW, mpudatag->Gyro_Y_RAW, mpudatag->Gyro_Z_RAW);

    // snprintf sonucu 0'dan bÃ¼yÃ¼k ve tampon boyutundan kÃ¼Ã§Ã¼k ise veriyi gÃ¶nder
    if (len > 0 && len < sizeof(buffer)) {
        HAL_UART_Transmit(&huart3, buffer, len, HAL_MAX_DELAY);
    }
}

void getGres() {
     switch (Gscale)
  {
  	  case GFS_250DPS:
      gRes = 250.0/32768.0;
      break;
      case GFS_500DPS:
      gRes = 500.0/32768.0;
      break;
      case GFS_1000DPS:
      gRes = 1000.0/32768.0;
      break;
      case GFS_2000DPS:
      gRes = 2000.0/32768.0;
      break;
  }
}

void getAres() {
     switch (Ascale)

  {
  	case AFS_2G:
    aRes = 2.0/32768.0;
    break;
    case AFS_4G:
    aRes = 4.0/32768.0;
    break;
    case AFS_8G:
    aRes = 8.0/32768.0;
    break;
    case AFS_16G:
    aRes = 16.0/32768.0;
    break;
 }
}

void getMres () {
  switch (Mscale)
  {
    case MFS_14BITS:
          mRes = 0.6;
          break;
    case MFS_16BITS:
          mRes = 10.*4912./32760.0;
          break;
  }
}
