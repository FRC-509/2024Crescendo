package com.redstorm509.alice2024.util.drivers;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;

/**
 * A driver for the Adafruit VL53L4CD Time-of-Flight Distance Sensor, adapted
 * from this Rust implementation: https://github.com/akeamc/vl53l4cd
 */
public class VL53L4CD {

	public enum Register {
		SOFT_RESET(0x0000),
		I2C_SLAVE_DEVICE_ADDRESS(0x0001),
		OSC_FREQ(0x0006),
		VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND(0x0008),
		MYSTERY_1(0x000B),
		XTALK_PLANE_OFFSET_KCPS(0x0016),
		XTALK_X_PLANE_GRADIENT_KCPS(0x0018),
		XTALK_Y_PLANE_GRADIENT_KCPS(0x001A),
		RANGE_OFFSET_MM(0x001E),
		INNER_OFFSET_MM(0x0020),
		OUTER_OFFSET_MM(0x0022),
		MYSTERY_2(0x0024),
		I2C_FAST_MODE_PLUS(0x002D),
		GPIO_HV_MUX_CTRL(0x0030),
		GPIO_TIO_HV_STATUS(0x0031),
		SYSTEM_INTERRUPT(0x0046),
		RANGE_CONFIG_A(0x005E),
		RANGE_CONFIG_B(0x0061),
		RANGE_CONFIG_SIGMA_THRESH(0x0064),
		MIN_COUNT_RATE_RTN_LIMIT_MCPS(0x0066),
		INTERMEASUREMENT_MS(0x006C),
		THRESH_HIGH(0x0072),
		THRESH_LOW(0x0074),
		SYSTEM_INTERRUPT_CLEAR(0x0086),
		SYSTEM_START(0x0087),
		RESULT_RANGE_STATUS(0x0089),
		RESULT_SPAD_NB(0x008C),
		RESULT_SIGNAL_RATE(0x008E),
		RESULT_AMBIENT_RATE(0x0090),
		RESULT_SIGMA(0x0092),
		RESULT_DISTANCE(0x0096),
		RESULT_OSC_CALIBRATE_VAL(0x00DE),
		FIRMWARE_SYSTEM_STATUS(0x00E5),
		IDENTIFICATION_MODEL_ID(0x010F);

		private final short address;

		Register(int address) {
			this.address = (short) address;
		}

		public short addr() {
			return address;
		}

		public byte[] asBytes() {
			return new byte[] { (byte) (address >> 8), (byte) address };
		}
	}

	public enum Status {
		Valid(0),
		SigmaAboveThreshold(1),
		SigmaBelowThreshold(2),
		DistanceBelowDetectionThreshold(3),
		InvalidPhase(4),
		HardwareFail(5),
		NoWrapAroundCheck(6),
		WrappedTargetPhaseMismatch(7),
		ProcessingFail(8),
		XTalkFail(9),
		InterruptError(10),
		MergedTarget(11),
		SignalTooWeak(12),
		Other(255);

		public static Status fromReturn(byte rtn) {
			switch (rtn) {
				case 3:
					return Status.HardwareFail;
				case 4:
				case 5:
					return Status.SigmaBelowThreshold;
				case 6:
					return Status.SigmaAboveThreshold;
				case 7:
					return Status.WrappedTargetPhaseMismatch;
				case 8:
					return Status.DistanceBelowDetectionThreshold;
				case 9:
					return Status.Valid;
				case 12:
					return Status.XTalkFail;
				case 13:
				case 18:
					return Status.InterruptError;
				case 19:
					return Status.NoWrapAroundCheck;
				case 22:
					return Status.MergedTarget;
				case 23:
					return Status.SignalTooWeak;
				default:
					return Status.Other;
			}
		}

		private final byte value;

		Status(int value) {
			this.value = (byte) value;
		}

		public byte getValue() {
			return value;
		}

		public Severity severity() {
			switch (this) {
				case Valid:
					return Severity.None;
				case SigmaAboveThreshold:
				case SigmaBelowThreshold:
					return Severity.Warning;
				case DistanceBelowDetectionThreshold:
				case InvalidPhase:
				case HardwareFail:
				case WrappedTargetPhaseMismatch:
				case ProcessingFail:
				case XTalkFail:
				case InterruptError:
				case MergedTarget:
				case SignalTooWeak:
				case Other:
					return Severity.Error;
				default:
					return Severity.None;
			}
		}
	};

	public enum Severity {
		None,
		Warning,
		Error
	}

	public class Measurement {
		public Status status;
		public short distanceMillimeters;
		public short ambientRate;
		public short signalRate;
		public short spadsEnabled;
		public short sigma;

		public Measurement(Status status, short distanceMillimeters, short ambientRate, short signalRate,
				short spadsEnabled, short sigma) {
			this.status = status;
			this.distanceMillimeters = distanceMillimeters;
			this.ambientRate = ambientRate;
			this.signalRate = signalRate;
			this.spadsEnabled = spadsEnabled;
			this.sigma = sigma;
		}

		public boolean isValid() {
			return status == Status.Valid;
		}
	}

	private static final class DefaultConfig {
		private static final byte[] MESSAGE = {
				// value addr : description
				0x12, // 0x2d : set bit 2 and 5 to 1 for fast plus mode (1MHz I2C), else don't
						// touch
				0x00, // 0x2e : bit 0 if I2C pulled up at 1.8V, else set bit 0 to 1 (pull up at
						// AVDD)
				0x00, // 0x2f : bit 0 if GPIO pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD)
				0x11, // 0x30 : set bit 4 to 0 for active high interrupt and 1 for active low (bits
						// 3:0 must be 0x1)
				0x02, // 0x31 : bit 1 = interrupt depending on the polarity
				0x00, // 0x32 : not user-modifiable
				0x02, // 0x33 : not user-modifiable
				0x08, // 0x34 : not user-modifiable
				0x00, // 0x35 : not user-modifiable
				0x08, // 0x36 : not user-modifiable
				0x10, // 0x37 : not user-modifiable
				0x01, // 0x38 : not user-modifiable
				0x01, // 0x39 : not user-modifiable
				0x00, // 0x3a : not user-modifiable
				0x00, // 0x3b : not user-modifiable
				0x00, // 0x3c : not user-modifiable
				0x00, // 0x3d : not user-modifiable
				(byte) 0xFF, // 0x3e : not user-modifiable
				0x00, // 0x3f : not user-modifiable
				0x0F, // 0x40 : not user-modifiable
				0x00, // 0x41 : not user-modifiable
				0x00, // 0x42 : not user-modifiable
				0x00, // 0x43 : not user-modifiable
				0x00, // 0x44 : not user-modifiable
				0x00, // 0x45 : not user-modifiable
				0x20, // 0x46 : interrupt configuration 0->level low detection, 1-> level high, 2->
						// Out of window, 3->In window, 0x20-> New sample ready , TBC
				0x0B, // 0x47 : not user-modifiable
				0x00, // 0x48 : not user-modifiable
				0x00, // 0x49 : not user-modifiable
				0x02, // 0x4a : not user-modifiable
				0x14, // 0x4b : not user-modifiable
				0x21, // 0x4c : not user-modifiable
				0x00, // 0x4d : not user-modifiable
				0x00, // 0x4e : not user-modifiable
				0x05, // 0x4f : not user-modifiable
				0x00, // 0x50 : not user-modifiable
				0x00, // 0x51 : not user-modifiable
				0x00, // 0x52 : not user-modifiable
				0x00, // 0x53 : not user-modifiable
				(byte) 0xC8, // 0x54 : not user-modifiable
				0x00, // 0x55 : not user-modifiable
				0x00, // 0x56 : not user-modifiable
				0x38, // 0x57 : not user-modifiable
				(byte) 0xFF, // 0x58 : not user-modifiable
				0x01, // 0x59 : not user-modifiable
				0x00, // 0x5a : not user-modifiable
				0x08, // 0x5b : not user-modifiable
				0x00, // 0x5c : not user-modifiable
				0x00, // 0x5d : not user-modifiable
				0x01, // 0x5e : not user-modifiable
				(byte) 0xCC, // 0x5f : not user-modifiable
				0x07, // 0x60 : not user-modifiable
				0x01, // 0x61 : not user-modifiable
				(byte) 0xF1, // 0x62 : not user-modifiable
				0x05, // 0x63 : not user-modifiable
				0x00, // 0x64 : Sigma threshold MSB (mm in 14.2 format for MSB+LSB), default value 90
						// mm
				(byte) 0xA0, // 0x65 : Sigma threshold LSB
				0x00, // 0x66 : Min count Rate MSB (MCPS in 9.7 format for MSB+LSB)
				(byte) 0x80, // 0x67 : Min count Rate LSB
				0x08, // 0x68 : not user-modifiable
				0x38, // 0x69 : not user-modifiable
				0x00, // 0x6a : not user-modifiable
				0x00, // 0x6b : not user-modifiable
				0x00, // 0x6c : Intermeasurement period MSB, 32 bits register
				0x00, // 0x6d : Intermeasurement period
				0x0F, // 0x6e : Intermeasurement period
				(byte) 0x89, // 0x6f : Intermeasurement period LSB
				0x00, // 0x70 : not user-modifiable
				0x00, // 0x71 : not user-modifiable
				0x00, // 0x72 : distance threshold high MSB (in mm, MSB+LSB)
				0x00, // 0x73 : distance threshold high LSB
				0x00, // 0x74 : distance threshold low MSB ( in mm, MSB+LSB)
				0x00, // 0x75 : distance threshold low LSB
				0x00, // 0x76 : not user-modifiable
				0x01, // 0x77 : not user-modifiable
				0x07, // 0x78 : not user-modifiable
				0x05, // 0x79 : not user-modifiable
				0x06, // 0x7a : not user-modifiable
				0x06, // 0x7b : not user-modifiable
				0x00, // 0x7c : not user-modifiable
				0x00, // 0x7d : not user-modifiable
				0x02, // 0x7e : not user-modifiable
				(byte) 0xC7, // 0x7f : not user-modifiable
				(byte) 0xFF, // 0x80 : not user-modifiable
				(byte) 0x9B, // 0x81 : not user-modifiable
				0x00, // 0x82 : not user-modifiable
				0x00, // 0x83 : not user-modifiable
				0x00, // 0x84 : not user-modifiable
				0x01, // 0x85 : not user-modifiable
				0x00, // 0x86 : clear interrupt, 0x01=clear
				0x00, // 0x87 : ranging, 0x00=stop, 0x40=start
		};

	}

	private static final byte PERIPHERAL_ADDR = 0x29;

	public static void writeWord(I2CUtil i2c, Register register, short value) {
		byte[] bytes = new byte[2];

		bytes[0] = (byte) ((value >> 8) & 0xFF);
		bytes[1] = (byte) (value & 0xFF);

		i2c.writeToAddress16bit(register.addr(), bytes);
	}

	private static int readDword(I2CUtil i2c, Register address) {
		byte[] bytes = new byte[4];
		i2c.readFromAddress16bit(address.addr(), (byte) 4, bytes);
		return ((bytes[0] & 0xFF) << 24) |
				((bytes[1] & 0xFF) << 16) |
				((bytes[2] & 0xFF) << 8) |
				(bytes[3] & 0xFF);
	}

	private static short readWord(I2CUtil i2c, Register address) {
		byte[] bytes = new byte[2];
		i2c.readFromAddress16bit(address.addr(), (byte) 2, bytes);
		return (short) ((bytes[0] << 8) | (bytes[1] & 0xFF));
	}

	private static byte readByte(I2CUtil i2c, Register address) {
		byte[] bytes = new byte[1];
		i2c.readFromAddress16bit(address.addr(), (byte) 1, bytes);
		return bytes[0];
	}

	private static void writeDword(I2CUtil i2c, Register register, int value) {
		byte[] bytes = new byte[4];
		bytes[0] = (byte) ((value >> 24) & 0xFF);
		bytes[1] = (byte) ((value >> 16) & 0xFF);
		bytes[2] = (byte) ((value >> 8) & 0xFF);
		bytes[3] = (byte) (value & 0xFF);
		i2c.writeToAddress16bit(register.addr(), bytes);
	}

	private static Pair<Short, Short> rangeConfigValues(int timingBudgetUs, short oscFreq) {
		// I didn't make these values up because I'm not a wizard.
		// https://github.com/stm32duino/VL53L4CD/blob/b64ff4fa877c3cf156e11639e5fa305208dd3be9/src/vl53l4cd_api.cpp#L370

		int macroPeriodUs = (2304 * (0x40000000 / (int) oscFreq)) >> 6;
		timingBudgetUs <<= 12;
		short first = calculateF(macroPeriodUs, timingBudgetUs, 16);
		short second = calculateF(macroPeriodUs, timingBudgetUs, 12);

		return Pair.of(first, second);
	}

	private static short calculateF(int macroPeriodUs, int timingBudgetUs, int x) {
		int msByte = 0;
		int tmp = macroPeriodUs * x;
		int lsByte = ((timingBudgetUs + (tmp >> 7)) / (tmp >> 6)) - 1;

		while ((lsByte & 0xffffff00) > 0) {
			lsByte >>= 1;
			msByte += 1;
		}

		return (short) ((msByte << 8) | (lsByte & 0xff));
	}

	private I2CUtil i2c;

	public VL53L4CD(I2C.Port port) {
		i2c = new I2CUtil(port, PERIPHERAL_ADDR);
	}

	public void changeDeviceAddress(byte newDeviceAddress) {
		// Set the new device address.
		i2c.writeToAddress16bit(Register.I2C_SLAVE_DEVICE_ADDRESS.addr(), newDeviceAddress);
		// Re-instantiate the I2CUtil instance for the reloacted device address.
		int port = i2c.getPort();
		i2c = new I2CUtil(port, newDeviceAddress);
	}

	public void init() {
		short id = readWord(i2c, Register.IDENTIFICATION_MODEL_ID);

		if (id != -5206) {
			DriverStation.reportError("[VL53L4CD] Error: Strange Device Id", false);
		}

		System.out.println("[VL53L4CD] Waiting for Boot...");

		while (readByte(i2c, Register.FIRMWARE_SYSTEM_STATUS) != 0x3) {
			Timer.delay(0.001);
		}

		System.out.println("[VL53L4CD] Successfully Booted!");

		i2c.writeToAddress16bit((short) 0x2d, DefaultConfig.MESSAGE);

		startRanging();
		stopRanging();
		i2c.writeToAddress16bit(Register.VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND.addr(), (byte) 0x09);
		i2c.writeToAddress16bit(Register.MYSTERY_1.addr(), (byte) 0);
		writeWord(i2c, Register.MYSTERY_2, (short) 0x500);

		setRangeTiming(20, 0);
	}

	public void setRangeTiming(int timingBudgetMs, int interMeasurementMs) {
		if (timingBudgetMs < 10 || timingBudgetMs > 200) {
			DriverStation.reportError("[VL53L4CD] Timing budget must be in range [10, 200]", false);
		}

		short oscFreq = readWord(i2c, Register.OSC_FREQ);
		if (oscFreq == 0) {
			DriverStation.reportError("[VL53L4CD] Oscillation frequency is zero.", false);
		}

		int timingBudgetUs = timingBudgetMs * 1000;
		if (interMeasurementMs == 0) {
			// continuous mode
			writeDword(i2c, Register.INTERMEASUREMENT_MS, 0);
			timingBudgetUs -= 2500;
		} else {
			if (timingBudgetMs < interMeasurementMs) {
				DriverStation.reportError(
						"[VL53L4CD] Timing budget must be greater than or equal to inter-measurement.", false);
			}

			// autonomous low power mode
			int clockPoll = (int) readDword(i2c, Register.RESULT_OSC_CALIBRATE_VAL) & 0x3ff;
			double inter_measurement_fac = 1.055 * (interMeasurementMs * clockPoll);
			writeDword(i2c, Register.INTERMEASUREMENT_MS, (int) inter_measurement_fac);

			timingBudgetUs -= 4300;
			timingBudgetUs /= 2;
		}

		Pair<Short, Short> ab = rangeConfigValues(timingBudgetUs, oscFreq);
		writeWord(i2c, Register.RANGE_CONFIG_A, ab.getFirst());
		writeWord(i2c, Register.RANGE_CONFIG_B, ab.getSecond());
	}

	public Measurement measure() {
		waitForMeasurement();
		Measurement measurement = readMeasurement();
		clearInterrupt();
		return measurement;
	}

	public void startTemperatureUpdate() {
		i2c.writeToAddress16bit(Register.VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND.addr(), (byte) 0x81);
		i2c.writeToAddress16bit(Register.MYSTERY_1.addr(), (byte) 0x92);
		i2c.writeToAddress16bit(Register.SYSTEM_START.addr(), (byte) 0x40);

		waitForMeasurement();
		clearInterrupt();
		stopRanging();

		i2c.writeToAddress16bit(Register.VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND.addr(), (byte) 0x09);
		i2c.writeToAddress16bit(Register.MYSTERY_1.addr(), (byte) 0);
	}

	public void waitForMeasurement() {
		for (int i = 0; i < 1000; i++) {
			if (hasMeasurement()) {
				return;
			}
			Timer.delay(0.001);
		}
		DriverStation.reportError("[VL53L4CD] Timed out while waiting for a measurement.", false);
	}

	public boolean hasMeasurement() {
		byte ctrl = readByte(i2c, Register.GPIO_HV_MUX_CTRL);
		byte status = readByte(i2c, Register.GPIO_TIO_HV_STATUS);
		return (status & 1) != (ctrl >> 4 & 1);
	}

	public Measurement readMeasurement() {
		byte status = (byte) (readByte(i2c, Register.RESULT_RANGE_STATUS) & 0x1f);

		return new Measurement(
				Status.fromReturn(status),
				readWord(i2c, Register.RESULT_DISTANCE),
				(short) (readWord(i2c, Register.RESULT_AMBIENT_RATE) * 8),
				(short) (readWord(i2c, Register.RESULT_SIGNAL_RATE) * 8),
				(short) (readWord(i2c, Register.RESULT_SPAD_NB) / 256),
				(short) (readWord(i2c, Register.RESULT_SIGMA) / 4));
	}

	public void clearInterrupt() {
		i2c.writeToAddress16bit(Register.SYSTEM_INTERRUPT_CLEAR.addr(), (byte) 0x01);
	}

	public void startRanging() {
		if (readWord(i2c, Register.INTERMEASUREMENT_MS) == (short) 0) {
			// autonomous mode
			i2c.writeToAddress16bit(Register.SYSTEM_START.addr(), (byte) 0x21);
		} else {
			// continuous mode
			i2c.writeToAddress16bit(Register.SYSTEM_START.addr(), (byte) 0x40);
		}

		waitForMeasurement();
		clearInterrupt();
	}

	public void stopRanging() {
		i2c.writeToAddress16bit(Register.SYSTEM_START.addr(), (byte) 0x00);
	}
}