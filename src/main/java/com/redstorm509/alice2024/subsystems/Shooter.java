package com.redstorm509.alice2024.subsystems;

import com.redstorm509.alice2024.Constants;
import com.redstorm509.alice2024.util.devices.VL53L4CD;
import com.redstorm509.alice2024.util.devices.VL53L4CD.Measurement;
import com.redstorm509.alice2024.util.math.Conversions;
import com.redstorm509.alice2024.util.math.GeometryUtils;
import com.redstorm509.stormkit.math.PositionTarget;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
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
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
	public enum IntakingState {
		Idle,
		IntakingNote,
		OuttakingNote,
	}

	private TalonFX shooterLeader = new TalonFX(15); // Labelled SHOOTERL
	private TalonFX shooterFollower = new TalonFX(16); // Labelled SHOOTERR

	private CANSparkMax indexer = new CANSparkMax(12, MotorType.kBrushed);
	// private VL53L4CD initialToF = new VL53L4CD(I2C.Port.kMXP, (byte) 0x52);
	// private VL53L4CD secondaryToF = new VL53L4CD(I2C.Port.kMXP, (byte) 0x7B);
	private boolean firstInstantToF = false;
	private boolean secondaryInstantToF = false;

	private IntakingState currentState = IntakingState.Idle;

	private TalonFX pivotLeader = new TalonFX(13); // Labelled PIVOTL
	private TalonFX pivotFollower = new TalonFX(14); // Labelled PIVOTR
	private CANcoder pivotEncoder = new CANcoder(17);

	private VoltageOut openLoop = new VoltageOut(0).withEnableFOC(false);
	private VelocityVoltage closedLoopVelocity = new VelocityVoltage(0).withEnableFOC(false);
	private PositionVoltage closedLoopPosition = new PositionVoltage(0).withEnableFOC(false);

	private PositionTarget pivotTarget;
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
		pivotEncoderConf.MagnetSensor.MagnetOffset = Constants.Shooter.kPivotMagnetOffset / 360.0;
		pivotEncoderConf.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
		pivotEncoderConf.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
		pivotEncoder.getConfigurator().apply(pivotEncoderConf);

		pivotFollower.setControl(new Follower(pivotLeader.getDeviceID(), true));
		shooterFollower.setControl(new Follower(shooterLeader.getDeviceID(), true));

		// THIS IS ALSO VERY WRONG
		double absPosition = Conversions.degreesToFalcon(
				pivotEncoder.getAbsolutePosition().waitForUpdate(1).getValueAsDouble() * 360.0,
				Constants.Shooter.kPivotGearRatio);
		pivotLeader.setPosition(absPosition);
		pivotFollower.setPosition(absPosition);

		pivotTarget = new PositionTarget(pivotEncoder.getAbsolutePosition().waitForUpdate(1).getValueAsDouble() * 360.0,
				Constants.Shooter.kMinPivot, Constants.Shooter.kMaxPivot, Constants.Shooter.kMaxPivotSpeed);
	}

	public void rawShootNote(double speed) {
		if (Math.abs(speed) <= 0.1) {
			indexer.set(Constants.Shooter.kIndexerSpinSpeed);
		} else {
			indexer.set(0);
		}
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

	public double getPivotDegrees() {
		return pivotEncoder.getAbsolutePosition().getValue() * 360.0;
	}

	public void setPivotDegrees(double targetDegrees) {
		double delta = (targetDegrees - getPivotDegrees()) % 360;
		if (delta > 180.0d) {
			delta -= 360.0d;
		} else if (delta < -180.0d) {
			delta += 360.0d;
		}

		double target = getPivotDegrees() + delta;
		double ticks = Conversions.degreesToFalcon(target, Constants.Shooter.kPivotGearRatio);

		pivotLeader.setControl(closedLoopPosition.withPosition(ticks));

		pivotTarget.setTarget(target);
	}

	public void setPivotOutput(double percentOutput) {

		double previous = pivotTarget.getTarget();
		pivotTarget.update(percentOutput);

		/*-
		This is where we will add softstops
		if (percentOutput < 0.0d && !isValidState(pivotTarget.getTarget(), getArmLength())) {
			pivotTarget.setTarget(previous);
		}*/

		double delta = (pivotTarget.getTarget() - getPivotDegrees()) % 360;
		if (delta > 180.0d) {
			delta -= 360.0d;
		} else if (delta < -180.0d) {
			delta += 360.0d;
		}

		double target = getPivotDegrees() + delta;

		setPivotDegrees(target);
	}

	public boolean hasIntaken() { // rename
		return currentState == IntakingState.IntakingNote;
	}

	@Override
	public void periodic() {
		// Logic check does any of this make sense???
		// Also there should then be a function that can tell this thing that a note has
		// left and transitions the state to Idle.

		/*-
		// run every update
		Measurement firstStage = initialToF.measure();
		Measurement secondStage = secondaryToF.measure();
		
		// changed names to logic better
		boolean firstWasToF = firstInstantToF;
		boolean secondaryWasToF = secondaryInstantToF;
		
		// updates the instant ToF readings
		if (firstStage.distanceMillimeters > Constants.Shooter.kToFNoteDetectionThreshold) {
			firstInstantToF = true;
		} else {
			firstInstantToF = false;
		}
		if (secondStage.distanceMillimeters > Constants.Shooter.kToFNoteDetectionThreshold) {
			firstInstantToF = true;
		} else {
			secondaryWasToF = false;
		}
		
		// If the first pass tripped, and the second pass JUST tripped, we are intaking.
		if (firstWasToF && !secondaryWasToF && secondaryInstantToF) {
			currentState = IntakingState.IntakingNote;
		
		} else if (secondaryWasToF && !secondaryInstantToF && firstInstantToF) { //
			// If the second pass JUST Un-tripped, and the first pass is still tripped, we
			// are
			// outtaking.
			currentState = IntakingState.OuttakingNote;
		}
		*/

		SmartDashboard.putNumber("PivotIntegrated",
				Conversions.falconToDegrees(pivotLeader.getPosition().getValue(), Constants.Shooter.kPivotGearRatio));
		SmartDashboard.putNumber("PivotIntegrated2",
				Conversions.falconToDegrees(pivotFollower.getPosition().getValue(), Constants.Shooter.kPivotGearRatio));
		SmartDashboard.putNumber("PivotAbsolute", pivotEncoder.getAbsolutePosition().getValue() * 360.0);
	}
}
