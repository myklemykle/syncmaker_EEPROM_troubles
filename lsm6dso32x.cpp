#include "config.h"

// try to skip the entire file if this is not defined, cuz Arduino always compiles everything...
#ifdef IMU_LSM6DSO32X

// Paul's NXPSensorFusion library has a nice general-purpose
// structure, so i've been adapting it to other sensors as I've
// switched chips a few times ...


#include "lsm6dso32x-pid/lsm6dso32x_reg.h"

#include "lsm6dso32x.h"
// #include <EEPROM.h>
// #include <util/crc16.h>
// #include <elapsedMillis.h>

#define NXP_MOTION_CAL_EEADDR  60
#define NXP_MOTION_CAL_SIZE    68

#include <SPI.h>
// defines SPI and SPI1; our IMU is on SPI1
// SPIPORT is defined in config.h ...

// NOTE: 
// "chip_addr" in SPI mode is the number of the GPIO pin that asserts CS for the chip --
// kind of squeezing SPI's foot into I2C's shoe, but it fits.
	
const uint8_t chip_addr=SPI_cs;
// from datasheet:
const SPISettings spiconf(10000000, MSBFIRST, SPI_MODE3);


bool LSM6DSO32X_IMU::sleep() {
	//TODO
	return true;
}
bool LSM6DSO32X_IMU::wake() {
	//TODO
	return true;
}
bool LSM6DSO32X_IMU::available() {
	update();
	return true;
}

void LSM6DSO32X_IMU::readMotionSensor(int& ax, int& ay, int& az, int& gx, int& gy, int& gz) {
	update();
	ax = accel_mag_raw[0];
	ay = accel_mag_raw[1];
	az = accel_mag_raw[2];
	gx = gyro_raw[0];
	gy = gyro_raw[1];
	gz = gyro_raw[2];
}

// not using ...
//
// #define G_PER_COUNT            0.0001220703125f  // = 1/8192
// #define UT_PER_COUNT           0.1f
// #define DEG_PER_SEC_PER_COUNT  0.0625f  // = 1/16
//
// void LSM6DSO32X_IMU::readMotionSensor(float& ax, float& ay, float& az, float& gx, float& gy, float& gz) {
// 	update();
// 	ax = (float)accel_mag_raw[0] * G_PER_COUNT;
// 	ay = (float)accel_mag_raw[1] * G_PER_COUNT;
// 	az = (float)accel_mag_raw[2] * G_PER_COUNT;
// 	gx = (float)accel_mag_raw[3] * DEG_PER_SEC_PER_COUNT;
// 	gy = (float)accel_mag_raw[4] * DEG_PER_SEC_PER_COUNT;
// 	gz = (float)accel_mag_raw[5] * DEG_PER_SEC_PER_COUNT;
// }
//
// #undef G_PER_COUNT
// #undef DEG_PER_SEC_PER_COUNT
// #undef UT_PER_COUNT

bool LSM6DSO32X_IMU::begin(){
	uint8_t buf;
  uint8_t temp; // actually signed ...
  uint8_t tempfrac;

  Serial.print("LSM6DSO32X::begin: SPI slave on pin ");
  Serial.println(SPI_cs);

	SPIPORT.begin(false); // we seem to need to manage CS ourselves.

  // detect if chip is present
  if (!read_regs(SPI_cs, LSM6DSO32X_WHO_AM_I, &buf, 1)) return false;
  Serial.printf("LSM6DSO32X ID = %02X\n", buf);
  if ( (buf != 0x6CU) )
    return false;


  // // reset the device:
  // ctrl3_c: "reboot memory" (first bit) and "software reset" (last bit)
	// interrupts active low, defaults otherwise.
  if (!write_reg(SPI_cs, LSM6DSO32X_CTRL3_C, 0b10100101)) return false;

  // wait 1ms for reset to complete
  delay(2);
  // detect if chip is still with us
  if (!read_regs(SPI_cs, LSM6DSO32X_WHO_AM_I, &buf, 1)) return false;
  if ( (buf != 0x6CU) ) {
    Serial.println("gone after reset!");
    return false;
	} else {
    Serial.println("reset unit");
	}


  // get the temperature: OUT_TEMP_L/H
  // H is a signed integer celcius value, L is unsigned fractional 1/256ths of a degree celcius
  if (!read_regs(SPI_cs, LSM6DSO32X_OUT_TEMP_H, &temp, 1)) return false;
  if (!read_regs(SPI_cs, LSM6DSO32X_OUT_TEMP_L, &tempfrac, 1)) return false;
  Serial.printf("temperature raw %i . %i\n", (int8_t)temp, tempfrac);
	// datasheet sec 4.3, table 4, teeny tiny footnote #3: temp sensor output is zero at 25 celcius ...
  Serial.printf("temperature %f\n", ((int8_t)temp + 25 + ( tempfrac / 256.0)));

  // configure components:

  // disable FIFO
  if (!write_reg(SPI_cs, LSM6DSO32X_FIFO_CTRL4, 0)) return false;

  // accel data rate & resolution: CTRL1_XL
#ifdef IMU_8KHZ
  // rate = 6.66khz, res = +-8g
  if (!write_reg(SPI_cs, LSM6DSO32X_CTRL1_XL, 0b10101000)) return false;
#else
  // rate = 1.66khz, res = +-8g
  if (!write_reg(SPI_cs, LSM6DSO32X_CTRL1_XL, 0b10001000)) return false;
#endif

  // gyro data rate & resolution:
#ifdef IMU_8KHZ
  // rate = 6.66khz, res = +-1000 deg/sec
  if (!write_reg(SPI_cs, LSM6DSO32X_CTRL2_G, 0b10101000)) return false;
#else
  // rate = 1.66khz, res = +-8g
  if (!write_reg(SPI_cs, LSM6DSO32X_CTRL2_G, 0b10001000)) return false;
#endif


#ifdef IMU_INTERRUPTS
  // configure interrupt 1 on accelerometer data ready
  if (!write_reg(SPI_cs, LSM6DSO32X_INT1_CTRL, 0b00000001)) return false;
	
#endif

  Serial.println("LSM6DSO32X configured");
  return true;
}

void LSM6DSO32X_IMU::update()
{
	// static elapsedMillis msec;
	int32_t alt;

	if (LSM6DSO32X_read(accel_mag_raw)) { // accel + mag
		//Serial.println("accel+mag");
		// newdata = 1;
	}
}

// TOOO: if we don't use gyro data, we could add a method to just get accel.
bool LSM6DSO32X_IMU::LSM6DSO32X_read(int16_t *data)  // accel + mag
{
  uint8_t buf[13];

	// TODO: its nutty that the chip organizes the high & low bytes backwards ...
	// nevertheless I could just clock out two bytes and swap them,
	// which would be 25% less bus traffic
  if (!read_regs(chip_addr, LSM6DSO32X_OUTX_H_A, buf+1, 1)) return false;
  if (!read_regs(chip_addr, LSM6DSO32X_OUTX_L_A, buf+2, 1)) return false;
  if (!read_regs(chip_addr, LSM6DSO32X_OUTY_H_A, buf+3, 1)) return false;
  if (!read_regs(chip_addr, LSM6DSO32X_OUTY_L_A, buf+4, 1)) return false;
  if (!read_regs(chip_addr, LSM6DSO32X_OUTZ_H_A, buf+5, 1)) return false;
  if (!read_regs(chip_addr, LSM6DSO32X_OUTZ_L_A, buf+6, 1)) return false;
  if (!read_regs(chip_addr, LSM6DSO32X_OUTX_H_G, buf+7, 1)) return false;
  if (!read_regs(chip_addr, LSM6DSO32X_OUTX_L_G, buf+8, 1)) return false;
  if (!read_regs(chip_addr, LSM6DSO32X_OUTY_H_G, buf+9, 1)) return false;
  if (!read_regs(chip_addr, LSM6DSO32X_OUTY_L_G, buf+10, 1)) return false;
  if (!read_regs(chip_addr, LSM6DSO32X_OUTZ_H_G, buf+11, 1)) return false;
  if (!read_regs(chip_addr, LSM6DSO32X_OUTZ_L_G, buf+12, 1)) return false;

  data[0] = (int16_t)((buf[1] << 8) | buf[2]);
  data[1] = (int16_t)((buf[3] << 8) | buf[4]);
  data[2] = (int16_t)((buf[5] << 8) | buf[6]);
  data[3] = (int16_t)((buf[7] << 8) | buf[8]);
  data[4] = (int16_t)((buf[9] << 8) | buf[10]);
  data[5] = (int16_t)((buf[11] << 8) | buf[12]);

	return true;
}

bool LSM6DSO32X_IMU::write_reg(uint8_t selector, uint8_t addr, uint8_t val)
{
	digitalWrite(selector, LOW);
	SPIPORT.transfer(addr & 0b01111111); // first bit clear for write-op
	SPIPORT.transfer(val);
	digitalWrite(selector, HIGH);
	return true; // what to check actually?
}

bool LSM6DSO32X_IMU::read_regs(uint8_t selector, uint8_t addr, uint8_t *data, uint8_t num)
{
	SPIPORT.beginTransaction(spiconf);
	digitalWrite(selector, LOW);
	SPIPORT.transfer(addr | 0b10000000); // first bit set for read-op
	while (num > 0) {
		*data++ = SPIPORT.transfer(0x00);
		num--;
	}
	digitalWrite(selector, HIGH);
	SPIPORT.endTransaction();
	return true;
}


// bool LSM6DSO32X_IMU::writeCalibration(const void *data)
// {
// 	const uint8_t *p = (const uint8_t *)data;
// 	uint16_t crc;
// 	uint8_t i;
//
// 	if (p[0] != 117 || p[1] != 84) return false;
// 	crc = 0xFFFF;
// 	for (i=0; i < NXP_MOTION_CAL_SIZE; i++) {
// 		crc = _crc16_update(crc, p[i]);
// 	}
// 	if (crc != 0) return false;
// 	for (i=0; i < NXP_MOTION_CAL_SIZE; i++) {
// 		EEPROM.write(NXP_MOTION_CAL_EEADDR + i, p[i]);
// 	}
// 	for (i=0; i < NXP_MOTION_CAL_SIZE; i++) {
// 		if (EEPROM.read(NXP_MOTION_CAL_EEADDR + i) != p[i]) return false;
// 	}
// 	memcpy(cal, ((const uint8_t *)data)+2, sizeof(cal));
// 	return true;
// }
//




#endif
