package com.redstorm509.alice2024;

import com.redstorm509.alice2024.util.math.Conversions;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class Constants {
	// Set to true if in replay mode. Set to false if in simulation mode.
	public static final boolean kAdvantageKitReplay = true;

	public static final String kRio = "rio";
	public static final String kCANIvore = "509CANIvore";
	public static final double kStickDeadband = 0.1;

	public static class Chassis {
		public static final double kOffsetToSwerveModule = Units.inchesToMeters(10.375);
	}

	public static class MK4I {
		public static final double kWheelCircumference = Units.inchesToMeters(4.0) * Math.PI; // 0.3192 meters
		public static final double kDriveGearRatio = 425.0d / 63.0d;
		public static final double kAngleGearRatio = 150.0d / 7.0d;
		public static final double kCouplingRatio = 25.0d / 7.0d;
	}

	public static class Shooter {
		// Replace Me!
		public static final Translation2d kPointOfRotation = new Translation2d();
		public static final Translation2d kOffsetToShootingPoint = new Translation2d();
	}

	public static class PreCompressor {
	}

	public static class Intake {
	}

	public static class Vision {
		// Replace Me!
		public static final Pose3d kIntakeCameraPose = new Pose3d();
		public static final Pose3d kShooterCameraPose = new Pose3d();
	}

	// Replace Me!
	public static final double kMaxSpeed = Conversions.falconToMPS(6380.0d / 60.0d, MK4I.kWheelCircumference,
			MK4I.kDriveGearRatio);

	// Replace Me!
	public static final double kMaxAngularVelocity = kMaxSpeed
			/ (Math.hypot(Chassis.kOffsetToSwerveModule, Chassis.kOffsetToSwerveModule));
	public static final double kMaxAngularAcceleration = 0.0;

	// Tune Me!
	public static final double kDriveVelocityS = 0.0 / 12.0;
	public static final double kDriveVelocityV = 0.0 / 12.0;
	public static final double kDriveVelocityA = 0.0 / 12.0;

	// Tune Me!
	public static final double kDriveVelocityP = 0.0;
	public static final double kDriveVelocityI = 0.0;
	public static final double kDriveVelocityD = 0.0;

	// Tune Me!
	public static final double kSteerAngleP = 0.0;
	public static final double kSteerAngleI = 0.0;
	public static final double kSteerAngleD = 0.0;

	// Tune Me!
	public static final double kHeadingPassiveP = 0.0 * 12.0d;
	public static final double kHeadingPassiveI = 0.0 * 12.0d;
	public static final double kHeadingPassiveD = 0.0 * 12.0d;
	public static final double kHeadingAggressiveP = 0.0 * 12.0d;
	public static final double kHeadingAggressiveI = 0.0 * 12.0d;
	public static final double kHeadingAggressiveD = 0.0 * 12.0d;
	public static final double kHeadingTimeout = 0.35;
	public static final double kMinHeadingCorrectionSpeed = 0.15;

	public static record SwerveModuleConfiguration(
			int moduleNumber,
			int angleEncoderId,
			int angleMotorId,
			int driveMotorId,
			double angleEncoderOffset) {
	}

	// Replace Me!
	public static final SwerveModuleConfiguration kFrontRight = new SwerveModuleConfiguration(
			0,
			0,
			0,
			0,
			0);

	public static final SwerveModuleConfiguration kFrontLeft = new SwerveModuleConfiguration(
			1,
			0,
			0,
			0,
			0);

	public static final SwerveModuleConfiguration kBackLeft = new SwerveModuleConfiguration(
			2,
			0,
			0,
			0,
			0);

	public static final SwerveModuleConfiguration kBackRight = new SwerveModuleConfiguration(
			3,
			0,
			0,
			0,
			0);
}