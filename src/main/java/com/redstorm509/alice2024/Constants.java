package com.redstorm509.alice2024;

import com.redstorm509.alice2024.util.math.Conversions;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class Constants {
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
	public static class Arm {
		public static final Translation3d kPointOfRotation = new Translation3d(-0.253997, 0, 0.581834);
		public static final Translation3d kDefaultShootingOrigin = new Translation3d(0.019505, 0, 0.597487);
		// public static final double kPivotToShootAngleOffset = 0.0d;
		// Replace Me!
		public static final Translation3d kGoalApex = new Translation3d(0.1, 0, Units.inchesToMeters(80.5) + 0.3);
		public static final double kMinPivot = -58.166;
		public static final double kMaxPivot = 71.834;
		public static final double kMaxPivotSpeed = 250.0d;

		public static final double kPivotMagnetOffset = -145.880844;

		public static final double kPivotIntegratedP = 5.1;
		public static final double kPivotIntegratedI = 0.72;
		public static final double kPivotIntegratedD = 0.24;

		public static final double kPivotRSP = 8 * 12;
		public static final double kPivotRSI = 0 * 12;
		public static final double kPivotRSD = 0.48 * 12;

		public static final double kPivotGearRatio = 168.719419;// 512.0d / 3.0d;
	}

	public static class Shooter {
		public static final double kFlyWheelP = 0.1;
		public static final double kFlyWheelI = 0.0;
		public static final double kFlyWheelD = 0.0;
		public static final double kFlyWheelS = 0.155;
		public static final double kFlyWheelV = 0.11;
		public static final double kFlyWheelA = 0.0;

		public static final double kTargetSpeed = kFalconFreeSpeedRPS * 1.0;
		public static final double kFlyWheelCircumference = Units.inchesToMeters(4) * Math.PI; // 0.3192 meters
	}

	public static class Indexer {
		public static final double kShootSpeed = -1.0;
		public static final double kSpinSpeed = 0.55; // 0.7
		public static final double kReducedSpinSpeed = 0.4;
	}

	public static class Intake {
		public static final double kIntakeSpinSpeed = 0.3;
		public static final double kPreCompressorSpinSpeed = 0.8;
		public static final double kIntermediateStageSpinSpeed = 0.8;
	}

	public static class Vision {
		public static class Pipeline {
			public static final int NeuralNetwork = 1;
			public static final int AprilTags = 0; // works for both, (intake ll needs to be set to pipeline 1)
		};

		public static final double kLimelightFullFOVAngle = 24.85 * 2;

		public static final double kIntakeCameraAngleOffset = -5.0;
		public static final double kIntakeCameraHeightFromGround = 0.2794; // meters

		public static final double kShoooterCameraAngleOffset = 25.0;
		public static final double kShoooterCameraHeightFromGround = 0.38735;

		public static final double kMaxTargetDistanceVariation = 0.2; // meters
		// REPLACE ME D:<
		public static final double kAlignmentTranslationTolerance = 0.0; // meters
		public static final double kAlignmentRotationTolerance = 0.0; // degrees
	}

	public static class Climber {
		// REPLACE ME >:((((
		public static final double kMaxExtensionLength = 0.0; // milimeters
		public static final double kAcceptableRoll = 0.0; // degrees
		public static final double kMaxRollCompensationAngle = 5.0; // degrees

		public static final double kExtensionP = 0.0;
		public static final double kExtensionI = 0.0;
		public static final double kExtensionD = 0.0;
	}

	public static final double kFalconFreeSpeedRPS = 6380.0d / 60.0d;
	public static final double kMaxSpeed = Conversions.falconToMPS(kFalconFreeSpeedRPS, MK4I.kWheelCircumference,
			MK4I.kDriveGearRatio);

	public static final double kMaxAngularVelocity = kMaxSpeed
			/ (Math.hypot(Chassis.kOffsetToSwerveModule, Chassis.kOffsetToSwerveModule));
	public static final double kMaxAngularAcceleration = 0.0;

	public static final double kDriveVelocityS = 0.124;
	public static final double kDriveVelocityV = 0.109;
	public static final double kDriveVelocityA = 0.0;

	public static final double kDriveVelocityP = 0.2;
	public static final double kDriveVelocityI = 3.0;
	public static final double kDriveVelocityD = 0.0;

	public static final double kSteerAngleP = 100.0;
	public static final double kSteerAngleI = 0.0;
	public static final double kSteerAngleD = 0.0;

	// Tune Me!
	public static final double kHeadingPassiveP = 8.0;
	public static final double kHeadingPassiveI = 0.15;
	public static final double kHeadingPassiveD = 0.4;
	public static final double kHeadingAggressiveP = 11.5;
	public static final double kHeadingAggressiveI = 0.25;
	public static final double kHeadingAggressiveD = 0.8;
	public static final double kHeadingTimeout = 0.25;
	public static final double kMinHeadingCorrectionSpeed = 0.05;

	public static record SwerveModuleConfiguration(
			int moduleNumber,
			int steerEncoderId,
			int steerMotorId,
			int driveMotorId,
			double steerEncoderOffset) {
	}

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