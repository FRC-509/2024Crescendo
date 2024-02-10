package com.redstorm509.alice2024.subsystems;

import com.redstorm509.alice2024.Constants;
import com.redstorm509.alice2024.util.math.Conversions;
import com.redstorm509.alice2024.util.math.GeometryUtils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
	private TalonFX shooterLeader = new TalonFX(0);
	private TalonFX shooterFollower = new TalonFX(0);

	private TalonFX indexer = new TalonFX(0);

	private TalonFX pivotLeader = new TalonFX(0);
	private TalonFX pivotFollower = new TalonFX(0);
	private CANcoder pivotEncoder = new CANcoder(0);

	private VoltageOut openLoop = new VoltageOut(0).withEnableFOC(false);
	private VelocityVoltage closedLoopVelocity = new VelocityVoltage(0).withEnableFOC(false);
	private PositionVoltage closedLoopPosition = new PositionVoltage(0).withEnableFOC(false);

	// private double pivotTargetDegrees;

	// TODO: Define coordinate space!
	public Shooter() {
		TalonFXConfiguration shootConf = new TalonFXConfiguration();
		shootConf.CurrentLimits.StatorCurrentLimitEnable = true;
		shootConf.CurrentLimits.StatorCurrentLimit = 35.0;
		shooterLeader.getConfigurator().apply(shootConf);
		shooterFollower.getConfigurator().apply(shootConf);

		TalonFXConfiguration pivotConf = new TalonFXConfiguration();
		pivotLeader.getConfigurator().apply(pivotConf);
		pivotFollower.getConfigurator().apply(pivotConf);

		CANcoderConfiguration pivotEncoderConf = new CANcoderConfiguration();
		pivotEncoderConf.MagnetSensor.MagnetOffset = 0.0;
		pivotEncoderConf.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;

		pivotFollower.setControl(new Follower(pivotLeader.getDeviceID(), true));
		shooterFollower.setControl(new Follower(shooterLeader.getDeviceID(), true));
	}

	public void rawShootNote(double speed) {
		indexer.setControl(openLoop.withOutput(speed * 12.0));
		shooterLeader.setControl(openLoop.withOutput(speed * 12.0));
	}

	// Gets the point of shooting relative to the origin of the robot.
	private Translation3d getShootingOrigin() {
		// I don't understand why a negative sign is needed; its probably because 3D
		// rotations need to pay attention to an AoR's direction.
		return GeometryUtils.rotatePointAboutPoint3D(Constants.Shooter.kDefaultShootingOrigin,
				Constants.Shooter.kPointOfRotation,
				new Rotation3d(0, Math.toRadians(-getPivotDegrees()), 0));
	}

	private double getOptimalAngleRad(Translation2d distanceFromShooterToApexMeters) {
		double d = distanceFromShooterToApexMeters.getX();
		double h = distanceFromShooterToApexMeters.getY();
		return Math.atan2(2 * h, d);
	}

	private double getOptimalVelocityMPS(double thetaRadians, Translation2d distanceFromShooterToApexMeters) {
		return Math.sqrt(
				(2 * Constants.kGravity * distanceFromShooterToApexMeters.getY())
						/ (Math.sin(thetaRadians) * Math.sin(thetaRadians)));
	}

	public void shooterMath(Pose3d robotPose) {
		// I have no idea if this works.
		Translation3d shootingOriginFieldSpace = robotPose.getTranslation()
				.plus(getShootingOrigin().rotateBy(robotPose.getRotation()));

		// Get "horziontal" and "vertical" distance between the shooting origin and the
		// desired apex.
		double zDistance = Constants.Shooter.kGoalApex.getZ() - shootingOriginFieldSpace.getZ();
		double xyDistance = Constants.Shooter.kGoalApex.toTranslation2d()
				.getDistance(shootingOriginFieldSpace.toTranslation2d());
		Translation2d distanceFromShooterToApex = new Translation2d(xyDistance, zDistance);

		// Get optimal target angle and set pivot to it.
		double optimalTargetAngle = getOptimalAngleRad(distanceFromShooterToApex);
		setPivotDegrees(MathUtil.clamp(Math.toDegrees(optimalTargetAngle) - Constants.Shooter.kPivotToShootAngleOffset,
				Constants.Shooter.kMinPivot, Constants.Shooter.kMaxPivot));

		// Get required velocity in meters per second.
		double targetVelocity = getOptimalVelocityMPS(optimalTargetAngle, distanceFromShooterToApex);

		// ??? profit
	}

	public double getPivotDegrees() {
		return pivotEncoder.getAbsolutePosition().getValue() * 360.0;
	}

	public void setPivotDegrees(double targetDegrees) {
		// pivotTargetDegrees = getPivotDegrees() - targetDegrees;
	}

	public void setPivotOutput(double percentOutput) {
		pivotLeader.setControl(openLoop.withOutput(percentOutput * 12.0));
	}

	@Override
	public void periodic() {
		/*
		 * CLAMPS BEWTEEN MIN AND MAX WITH ONLY RAW OUTPUT
		 * if (getPivotDegrees() > Constants.Shooter.kMaxPivot) {
		 * setPivotOutput(0.1);
		 * } else if (getPivotDegrees() < Constants.Shooter.kMinPivot) {
		 * setPivotOutput(0.1);
		 * }
		 */
		SmartDashboard.putNumber("PivotIntegrated", pivotLeader.getPosition().getValue());
		SmartDashboard.putNumber("PivotAbsolute", pivotEncoder.getAbsolutePosition().getValue());
	}
}
