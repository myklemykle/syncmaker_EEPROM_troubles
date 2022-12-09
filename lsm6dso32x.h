// try to skip the entire file if this is not defined, cuz Arduino always compiles everything...
#ifdef IMU_LSM6DSO32X

#ifndef _LSM6DSO_Motion_Sensors_
#define _LSM6DSO_Motion_Sensors_

/*
 * This has very little of the original NXP Sensor Fusion code from PJRC in it,
 * but that's where it started & why it doesn't perfectly fit the underlying hardware.
 * Still, easier to keep this interface for now.
 */

class LSM6DSO32X_IMU {
public:
	static const int countPerG = 4096;
	bool begin();
	bool sleep();
	bool wake();
	bool available();
	void readMotionSensor(int& ax, int& ay, int& az, int& gx, int& gy, int& gz);
	// not using:
	// void readMotionSensor(float& ax, float& ay, float& az, float& gx, float& gy, float& gz);

	// TODO: if we're not going to use the gyro data, 
	// don't waste time fetching it.  Add method to just get accel data.
private:
	void update();

	bool LSM6DSO32X_begin();
	bool LSM6DSO32X_read(int16_t *data);
	bool LSM6DSO32X_sleep();
	bool LSM6DSO32X_wake();
	static bool read_regs(uint8_t selector, uint8_t addr, uint8_t *data, uint8_t num);
	static bool write_reg(uint8_t selector, uint8_t addr, uint8_t val);
	static bool set_reg(uint8_t selector, uint8_t addr, uint8_t val);

	float cal[16]; // 0-8=offsets, 9=field strength, 10-15=soft iron map
	int16_t accel_mag_raw[6];
	int16_t gyro_raw[3];
	int16_t temperature_raw;
};

#endif
#endif
