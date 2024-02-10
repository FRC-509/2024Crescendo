package com.redstorm509.alice2024;

import com.redstorm509.alice2024.util.math.Conversions;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class Constants {
	// Set to true if in replay mode. Set to false if in simulation mode.
	public static final boolean kAdvantageKitReplay = false;

	public static final double kGravity = 9.80665;

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

	// (0, 0, 0) -> Center of Drivetrain Projected Onto Ground
	// Front of robot is the direction where the intake is pointing.
	public static class Shooter {
		// Replace Me!
		public static final Translation3d kPointOfRotation = new Translation3d(-0.253997, 0, 0.581834);
		public static final Translation3d kDefaultShootingOrigin = new Translation3d(0.019505, 0, 0.597487);
		public static final double kPivotToShootAngleOffset = 61.0d;
		public static final Translation3d kGoalApex = new Translation3d(0.1, 0, Units.inchesToMeters(80.5) + 0.3);
		public static final double kMinPivot = 0.0d; // MAKE SURE ARE CORRECT
		public static final double kMaxPivot = 130.0d;
		public static final double kMaxPivotSpeed = 0.0d; // REPLACE MEEE
	}

	public static class PreCompressor {
		public static final double kPreCompressorSpinSpeed = 0.7;
	}

	public static class Intake {
		public static final double kIntakeSpinSpeed = 0.35; // between -1.0 1.0
	}

	public static class Vision {
		public static final int NeuralNetworkPipeline = 0;
		public static final int AprilTagPipeline = 1;

		// Replace Me!
		public static final Pose3d kIntakeCameraPose = new Pose3d();
		public static final double kIntakeCameraAngleOffset = 5.0;
		public static final Pose3d kShooterCameraPose = new Pose3d();
		public static final double limelightFullFOVAngle = 24.85 * 2;
		public static final double kIntakeCameraHeightFromGround = 0.2794; // meters
	}

	public static class AprilTags {
		// NOTE: all offsets are from the center of the april tag
		public static class Amp {
			public static final double BlueAllianceTagID = 6;
			public static final double RedAllianceTagID = 5;
		}

		public static class Speaker {
			public static final double BlueAllianceCenterTagID = 7;
			public static final double BlueAllianceSideTagID = 8;
			public static final double RedAllianceCenterTagID = 4;
			public static final double RedAllianceSideTagID = 3;

			public static final double kDistanceToSpeaker = .87; // meters
			public static final double kHeightFromGround = 0.0; // Replace me if needed
		}

		public static class Trap {
		}
	}

	public static class Climber {
		// REPLACE ME >:((((
		public static final double kMaxExtensionLength = 0.0;
	}

	// Replace Me!
	public static final double kMaxSpeed = Conversions.falconToMPS(6380.0d / 60.0d, MK4I.kWheelCircumference,
			MK4I.kDriveGearRatio);

	// Replace Me!
	public static final double kMaxAngularVelocity = kMaxSpeed
			/ (Math.hypot(Chassis.kOffsetToSwerveModule, Chassis.kOffsetToSwerveModule));
	public static final double kMaxAngularAcceleration = 0.0;

	// Tune Me!
	public static final double kDriveVelocityS = 0.128;
	public static final double kDriveVelocityV = 0.109;
	public static final double kDriveVelocityA = 0.0;

	// Tune Me!
	public static final double kDriveVelocityP = 0.2;
	public static final double kDriveVelocityI = 3.0;
	public static final double kDriveVelocityD = 0.0;

	// Tune Me!
	public static final double kSteerAngleP = 100.0;
	public static final double kSteerAngleI = 0.0; // add
	public static final double kSteerAngleD = 0.0;

	// Tune Me!
	public static final double kHeadingPassiveP = 0.3 * 12.0d;
	public static final double kHeadingPassiveI = 0.8 * 12.0d;
	public static final double kHeadingPassiveD = 0.0 * 12.0d;
	public static final double kHeadingAggressiveP = 0.0 * 12.0d;
	public static final double kHeadingAggressiveI = 0.0 * 12.0d;
	public static final double kHeadingAggressiveD = 0.0 * 12.0d;
	public static final double kHeadingTimeout = 0.40;
	public static final double kMinHeadingCorrectionSpeed = 0.15;

	public static record SwerveModuleConfiguration(
			int moduleNumber,
			int steerEncoderId,
			int steerMotorId,
			int driveMotorId,
			double steerEncoderOffset) {
	}

	// Replace Me!, still??
	public static final SwerveModuleConfiguration kFrontRight = new SwerveModuleConfiguration(
			0,
			0,
			8,
			4,
			154.423828);

	public static final SwerveModuleConfiguration kFrontLeft = new SwerveModuleConfiguration(
			1,
			1,
			9,
			5,
			101.337891);

	public static final SwerveModuleConfiguration kBackLeft = new SwerveModuleConfiguration(
			2,
			2,
			10,
			6,
			178.066406);

	public static final SwerveModuleConfiguration kBackRight = new SwerveModuleConfiguration(
			3,
			3,
			11,
			7,
			258.925782);
}