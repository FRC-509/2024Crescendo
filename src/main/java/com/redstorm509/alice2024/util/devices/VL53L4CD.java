package com.redstorm509.alice2024.util.devices;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;

public class VL53L4CD {

	private static final class DefaultConfig {
		private static final byte[] MESSAGE = {
				0x00, // first byte of register to write to
				0x2d, // second byte of register to write to
				// value addr : description
				0x12, // 0x2d : set bit 2 and 5 to 1 for fast plus mode (1MHz I2C), else don't touch
				0x00, // 0x2e : bit 0 if I2C pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD)
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

	public enum Register {
		OSC_FREQ(0x0006),
		VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND(0x0008),
		MYSTERY_1(0x000b),
		MYSTERY_2(0x0024),
		SYSTEM_START(0x0087),
		GPIO_HV_MUX_CTRL(0x0030),
		GPIO_TIO_HV_STATUS(0x0031),
		RANGE_CONFIG_A(0x005e),
		RANGE_CONFIG_B(0x0061),
		INTERMEASUREMENT_MS(0x006c),
		SYSTEM_INTERRUPT_CLEAR(0x0086),
		RESULT_RANGE_STATUS(0x0089),
		RESULT_NUM_SPADS(0x008c),
		RESULT_SIGNAL_RATE(0x008e),
		RESULT_AMBIENT_RATE(0x0090),
		RESULT_SIGMA(0x0092),
		RESULT_DISTANCE(0x0096),
		RESULT_OSC_CALIBRATE_VAL(0x00de),
		SYSTEM_STATUS(0x00e5),
		IDENTIFICATION_MODEL_ID(0x010f);

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
	};

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

		private final byte value;

		Status(int value) {
			this.value = (byte) value;
		}

		public byte getValue() {
			return value;
		}

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
	}

	public enum Severity {
		None,
		Warning,
		Error
	}

	public class Measurement {
		public Status status;
		public short distanceMiliMeters;
		public short ambientRate;
		public short signalRate;
		public short spadsEnabled;
		public short sigma;

		public boolean isValid() {
			return status == Status.Valid;
		}
	}

	private static int readDword(I2C i2c, Register address) {
		byte[] bytes = new byte[4];
		i2c.read(address.addr(), 4, bytes);
		return ((bytes[0] & 0xFF) << 24) |
				((bytes[1] & 0xFF) << 16) |
				((bytes[2] & 0xFF) << 8) |
				(bytes[3] & 0xFF);
	}

	private static short readWord(I2C i2c, Register address) {
		byte[] bytes = new byte[2];
		i2c.read(address.addr(), 2, bytes);
		return (short) ((bytes[0] << 8) | (bytes[1] & 0xFF));
	}

	private static byte readByte(I2C i2c, Register address) {
		byte[] bytes = new byte[1];
		i2c.read(address.addr(), 1, bytes);
		return bytes[0];
	}

	private static void writeDword(I2C i2c, Register register, int value) {
		byte[] bytes = new byte[4];
		bytes[0] = (byte) ((value >> 24) & 0xFF);
		bytes[1] = (byte) ((value >> 16) & 0xFF);
		bytes[2] = (byte) ((value >> 8) & 0xFF);
		bytes[3] = (byte) (value & 0xFF);
		i2c.writeBulk(bytes);
	}

	public static void writeWord(I2C i2c, Register register, short value) {
		byte[] bytes = new byte[2];

		bytes[0] = (byte) ((value >> 8) & 0xFF);
		bytes[1] = (byte) (value & 0xFF);

		i2c.write(value, value);
	}

	private static final int PERIPHERAL_ADDR = 0x29;

	private I2C i2c;

	public VL53L4CD(I2C.Port port, int deviceAddr) {
		i2c = new I2C(port, deviceAddr);
	}

	public VL53L4CD(I2C.Port port) {
		i2c = new I2C(port, PERIPHERAL_ADDR);
	}

	public void init() {
		short id = readWord(i2c, Register.IDENTIFICATION_MODEL_ID);
		if (id != 0xEBAA) {
			DriverStation.reportError("[VL53L4CD] Error: Strange Device Id", null);
		}

		System.out.println("[VL53L4CD] Waiting for Boot...");

		while (readByte(i2c, Register.SYSTEM_STATUS) != 0x3) {
			Timer.delay(0.001);
		}

		System.out.println("[VL53L4CD] Successfully Booted!");

		i2c.writeBulk(DefaultConfig.MESSAGE);

		startRanging();
		stopRanging();
		i2c.write(Register.VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND.addr(), 0x09);
		i2c.write(Register.MYSTERY_1.addr(), 0);
		writeWord(i2c, Register.MYSTERY_2, (short) 0x500);

		setRangeTiming(50, 0);
	}

	private void stopRanging() {
	}

	private void startRanging() {
	}

	public void setRangeTiming(int timingBudgetMs, int interMeasurementMs) {
		if (timingBudgetMs < 10 || timingBudgetMs > 200) {
			DriverStation.reportError("[VL53L4CD] Timing budget must be in range [10, 200]", null);
		}

		short oscFreq = readWord(i2c, Register.OSC_FREQ);
		if (oscFreq == 0) {
			DriverStation.reportError("[VL53L4CD] Oscillation frequency is zero.", null);
		}

		int timingBudgetUs = timingBudgetMs * 1000;
		if (interMeasurementMs == 0) {
			// continuous mode
			writeDword(i2c, Register.INTERMEASUREMENT_MS, 0);
			timingBudgetUs -= 2500;
		} else {
			if (timingBudgetMs < interMeasurementMs) {
				DriverStation.reportError(
						"[VL53L4CD] Timing budget must be greater than or equal to inter-measurement.", null);
			}

			// autonomous low power mode
			int clockPoll = (int) readDword(i2c, Register.RESULT_OSC_CALIBRATE_VAL) & 0x3ff;
			double inter_measurement_fac = 1.055 * (interMeasurementMs * clockPoll);
			writeDword(i2c, Register.INTERMEASUREMENT_MS, (int) inter_measurement_fac);

			timingBudgetUs -= 4300;
			timingBudgetUs /= 2;
		}

		// let (a, b) = range_config_values(timing_budget_us, osc_freq);
		// self.i2c.writeWord(Register::RANGE_CONFIG_A, a).await?;
		// self.i2c.writeWord(Register::RANGE_CONFIG_B, b).await?;
	}

	public double getReading() {
		return 0.0d;
	}
}