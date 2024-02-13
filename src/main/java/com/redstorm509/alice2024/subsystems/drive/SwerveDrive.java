package com.redstorm509.alice2024.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.redstorm509.alice2024.Constants;
import com.redstorm509.alice2024.Constants.Chassis;

import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import com.redstorm509.stormkit.math.Interpolator;

public class SwerveDrive extends SubsystemBase {

	/*
	 * The order of each vector corresponds to the index of the swerve module inside
	 * the swerveModules array.
	 * 
	 * module 1 (-, +) |--f--| module 0 (+, +)
	 * module 2 (-, -) |--b--| module 3 (+, -)
	 */
	private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
			new Translation2d(+Chassis.kOffsetToSwerveModule, +Chassis.kOffsetToSwerveModule),
			new Translation2d(-Chassis.kOffsetToSwerveModule, +Chassis.kOffsetToSwerveModule),
			new Translation2d(-Chassis.kOffsetToSwerveModule, -Chassis.kOffsetToSwerveModule),
			new Translation2d(+Chassis.kOffsetToSwerveModule, -Chassis.kOffsetToSwerveModule));

	public SwerveModule[] swerveModules;
	public SwerveDriveOdometry odometry;
	private Field2d field2d;
	private Pigeon2 pigeon;

	private Interpolator headingInterplator;

	private Timer timer;

	private double prevTime = -1.0d;
	private double targetHeading;
	private PIDController headingPassive = new PIDController(Constants.kHeadingPassiveP, Constants.kHeadingPassiveI,
			Constants.kHeadingPassiveD);
	private PIDController headingAggressive = new PIDController(Constants.kHeadingAggressiveP,
			Constants.kHeadingAggressiveI, Constants.kHeadingAggressiveD);

	public SwerveDrive(Pigeon2 pigeon) {
		this.timer = new Timer();
		timer.start();

		this.pigeon = pigeon;
		this.headingInterplator = new Interpolator(Constants.kMaxAngularVelocity);
		this.targetHeading = 0.0;

		pigeon.setYaw(0.0d, 1.0d);

		swerveModules = new SwerveModule[] {
				new SwerveModule(Constants.kFrontRight),
				new SwerveModule(Constants.kFrontLeft),
				new SwerveModule(Constants.kBackLeft),
				new SwerveModule(Constants.kBackRight),
		};

		field2d = new Field2d();

		odometry = new SwerveDriveOdometry(kinematics, getYaw(), getModulePositions());

		HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
				new PIDConstants(5.0, 0, 0), // Translation constants
				new PIDConstants(5.0, 0, 0), // Rotation constants
				Constants.kMaxSpeed,
				Constants.Chassis.kOffsetToSwerveModule * Math.sqrt(2),
				new ReplanningConfig(false, false));

		AutoBuilder.configureHolonomic(
				this::getRawOdometeryPose,
				this::resetOdometry,
				this::getChassisSpeeds,
				this::setChassisSpeeds,
				pathFollowerConfig,
				() -> {
					// Boolean supplier that controls when the path will be mirrored for the red
					// alliance
					// This will flip the path being followed to the red side of the field.
					// THE ORIGIN WILL REMAIN ON THE BLUE SIDE

					Optional<Alliance> alliance = DriverStation.getAlliance();
					if (alliance.isPresent()) {
						return alliance.get() == DriverStation.Alliance.Red;
					}
					return false;
				},
				this);
		PathPlannerLogging.setLogActivePathCallback((poses) -> field2d.getObject("path").setPoses(poses));
		Shuffleboard.getTab("Robot Field Position").add(field2d);
	}

	public void drive(Translation2d translationMetersPerSecond, double rotationRadiansPerSecond, boolean fieldRelative,
			boolean omitRotationCorrection) {

		double dt = Timer.getFPGATimestamp() - prevTime;
		if (prevTime < 0) {
			dt = 0.02;
		}

		headingInterplator.setPoint(rotationRadiansPerSecond);

		double rotationOutput;

		double interpolatedRotation = headingInterplator.update(dt);

		boolean hasRotationInput = Math.abs(rotationRadiansPerSecond) > 0.01;

		if (hasRotationInput) {
			timer.reset();
		}

		double speed = Math.hypot(translationMetersPerSecond.getX(),
				translationMetersPerSecond.getY());

		if ((speed != 0 && speed < Constants.kMinHeadingCorrectionSpeed) || omitRotationCorrection || hasRotationInput
				|| timer.get() < Constants.kHeadingTimeout) {
			setTargetHeading(pigeon.getYaw().getValue());
			rotationOutput = interpolatedRotation;
		} else {
			double delta = pigeon.getYaw().getValue() - targetHeading;
			if (delta > 180.0d) {
				delta -= 360.0d;
			}
			if (delta < -180.0d) {
				delta += 360.0d;
			}

			// double outputDegrees = Math.abs(delta) > 2.0d ?
			// headingAggressive.calculate(delta) : headingPassive.calculate(delta);
			double outputDegrees = Math.abs(delta) > 0.5d ? headingPassive.calculate(delta) : 0;
			rotationOutput = Math.toRadians(outputDegrees);
		}

		SwerveModuleState[] moduleStates;

		if (fieldRelative) {
			moduleStates = kinematics
					.toSwerveModuleStates(ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(
							translationMetersPerSecond.getX(),
							translationMetersPerSecond.getY(),
							rotationOutput,
							getYaw()), dt));
		} else {
			moduleStates = kinematics.toSwerveModuleStates(new ChassisSpeeds(
					translationMetersPerSecond.getX(),
					translationMetersPerSecond.getY(),
					rotationOutput));
		}

		SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates,
				Constants.kMaxSpeed);

		for (SwerveModule mod : swerveModules) {
			mod.setDesiredState(moduleStates[mod.moduleNumber], true);
		}

		prevTime = Timer.getFPGATimestamp();
	}

	public void enterXStance() {
		swerveModules[0].setDesiredState(new SwerveModuleState(0.0d, Rotation2d.fromDegrees(45.0d)), false);
		swerveModules[1].setDesiredState(new SwerveModuleState(0.0d, Rotation2d.fromDegrees(135.0d)), false);
		swerveModules[2].setDesiredState(new SwerveModuleState(0.0d, Rotation2d.fromDegrees(45.0d)), false);
		swerveModules[3].setDesiredState(new SwerveModuleState(0.0d, Rotation2d.fromDegrees(135.0d)), false);
	}

	public void stopModules() {
		for (SwerveModule module : swerveModules) {
			module.setDesiredState(new SwerveModuleState(0, module.getAngle()), false);
		}
	}

	public void setTargetHeading(double heading) {
		targetHeading = heading % 360.0d;
	}

	public ChassisSpeeds getChassisSpeeds() {
		return kinematics.toChassisSpeeds(getModuleStates());
	}

	// Used strictly for PathPlanner in autonomous
	public void setChassisSpeeds(ChassisSpeeds robotRelativeSpeeds) {
		ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
		SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, Constants.kMaxSpeed);

		for (SwerveModule mod : swerveModules) {
			mod.setDesiredState(targetStates[mod.moduleNumber], true);
		}
	}

	public Pose2d getRawOdometeryPose() {
		return odometry.getPoseMeters();
	}

	public void resetOdometry(Pose2d pose) {
		odometry.resetPosition(getYaw(), getModulePositions(), pose);
	}

	public SwerveModule[] getModules() {
		return swerveModules;
	}

	public SwerveModuleState[] getModuleStates() {
		SwerveModuleState[] states = new SwerveModuleState[4];
		for (SwerveModule mod : swerveModules) {
			states[mod.moduleNumber] = mod.getState();
		}
		return states;
	}

	public SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] positions = new SwerveModulePosition[4];
		for (SwerveModule mod : swerveModules) {
			positions[mod.moduleNumber] = mod.getPosition();
		}
		return positions;
	}

	public Rotation2d getYaw() {
		return pigeon.getRotation2d();
	}

	@Override
	public void periodic() {
		odometry.update(getYaw(), getModulePositions());

		field2d.setRobotPose(getRawOdometeryPose());

		SmartDashboard.putNumber("yaw", getYaw().getDegrees());
		SmartDashboard.putNumber("target-heading", targetHeading);
		SmartDashboard.putNumber("odometry-x", getRawOdometeryPose().getX());
		SmartDashboard.putNumber("odometry-y", getRawOdometeryPose().getY());
		SmartDashboard.putNumber("odometry-theta", getRawOdometeryPose().getRotation().getDegrees());
	}
}