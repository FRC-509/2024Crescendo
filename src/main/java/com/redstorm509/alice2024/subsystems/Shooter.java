package com.redstorm509.alice2024.subsystems;

import com.redstorm509.alice2024.Constants;
import com.redstorm509.alice2024.util.devices.VL53L4CD;
import com.redstorm509.alice2024.util.devices.VL53L4CD.Measurement;
import com.redstorm509.alice2024.util.math.Conversions;
import com.redstorm509.alice2024.util.math.GeometryUtils;
import com.redstorm509.stormkit.math.PositionTarget;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import java.io.Console;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
	public enum IndexerState {
		Idle,
		IntakingNote,
		OuttakingNote,
		ShootingNote,
	}

	private static final double kAbsOffsetOffset = 50.0;

	private boolean hasNote = false;

	private TalonFX shooterLeader = new TalonFX(15); // Labelled SHOOTERL
	private TalonFX shooterFollower = new TalonFX(16); // Labelled SHOOTERR

	private CANSparkMax indexer = new CANSparkMax(12, MotorType.kBrushed);
	// private VL53L4CD initialToF;
	// private VL53L4CD secondaryToF;
	private boolean firstInstantToF = false;
	private boolean secondaryInstantToF = false;

	private IndexerState currentState = IndexerState.Idle;

	private TalonFX pivotLeader = new TalonFX(13); // Labelled PIVOTL
	private TalonFX pivotFollower = new TalonFX(14); // Labelled PIVOTR
	private CANcoder pivotEncoder = new CANcoder(17);
	private DigitalInput limitSwitch = new DigitalInput(0);

	private VoltageOut openLoop = new VoltageOut(0).withEnableFOC(false);
	private VelocityVoltage closedLoopVelocity = new VelocityVoltage(0).withEnableFOC(false);
	private PositionVoltage closedLoopPosition = new PositionVoltage(0).withEnableFOC(false);

	private PositionTarget pivotTarget;

	// TODO: Define coordinate space!
	public Shooter() {
		indexer.setSmartCurrentLimit(15);
		indexer.setIdleMode(IdleMode.kCoast);
		indexer.burnFlash();

		TalonFXConfiguration shootConf = new TalonFXConfiguration();
		shootConf.CurrentLimits.SupplyCurrentLimitEnable = true;
		shootConf.CurrentLimits.SupplyCurrentLimit = 35.0;
		shootConf.Slot0.kP = Constants.Shooter.kFlyWheelP;
		shootConf.Slot0.kI = Constants.Shooter.kFlyWheelI;
		shootConf.Slot0.kD = Constants.Shooter.kFlyWheelD;
		shootConf.Slot0.kS = Constants.Shooter.kFlyWheelS;
		shootConf.Slot0.kV = Constants.Shooter.kFlyWheelV;
		shootConf.Slot0.kA = Constants.Shooter.kFlyWheelA;
		shootConf.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		shooterLeader.getConfigurator().apply(shootConf);
		shooterFollower.getConfigurator().apply(shootConf);

		TalonFXConfiguration pivotConf = new TalonFXConfiguration();
		pivotConf.CurrentLimits.SupplyCurrentLimitEnable = true;
		pivotConf.CurrentLimits.SupplyCurrentLimit = 35.0;
		pivotConf.Slot0.kP = Constants.Shooter.kPivotP;
		pivotConf.Slot0.kI = Constants.Shooter.kPivotI;
		pivotConf.Slot0.kD = Constants.Shooter.kPivotD;
		pivotConf.MotorOutput.NeutralMode = NeutralModeValue.Coast;

		pivotLeader.getConfigurator().apply(pivotConf);
		pivotFollower.getConfigurator().apply(pivotConf);

		CANcoderConfiguration pivotEncoderConf = new CANcoderConfiguration();
		pivotEncoderConf.MagnetSensor.MagnetOffset = (Constants.Shooter.kPivotMagnetOffset + kAbsOffsetOffset) / 360.0;
		pivotEncoderConf.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
		pivotEncoderConf.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
		pivotEncoder.getConfigurator().apply(pivotEncoderConf);

		pivotFollower.setControl(new Follower(pivotLeader.getDeviceID(), true));
		shooterFollower.setControl(new Follower(shooterLeader.getDeviceID(), true));

		resetIntegratedToAbsolute(true);
		/*
		 * pivotLeader.setPosition(0);
		 * pivotFollower.setPosition(0);
		 * pivotEncoder.setPosition(0);
		 */
		pivotTarget = new PositionTarget(0, Constants.Shooter.kMinPivot, Constants.Shooter.kMaxPivot,
				Constants.Shooter.kMaxPivotSpeed);

		// Initialize the sensors one at a time to ensure that they both get unique
		// device addresses.
		// DigitalOutput initialTofXSHUT = new DigitalOutput(7);
		// DigitalOutput secondaryTofXSHUT = new DigitalOutput(8);
		// secondaryTofXSHUT.set(false);
		// initialTofXSHUT.set(false);
		// initialToF = new VL53L4CD(I2C.Port.kMXP);
		// initialToF.changeDeviceAddress((byte) 0x30);
		// secondaryToF = new VL53L4CD(I2C.Port.kMXP);
		// secondaryToF.changeDeviceAddress((byte) 0x31);
		// initialTof.init();
	}

	public void resetIntegratedToAbsolute(boolean waitForUpdate) {
		if (waitForUpdate) {
			double absPosition = Conversions.degreesToFalcon(
					pivotEncoder.getAbsolutePosition().waitForUpdate(1).getValueAsDouble() * 360.0 - kAbsOffsetOffset,
					Constants.Shooter.kPivotGearRatio);
			pivotLeader.setPosition(absPosition);
			pivotFollower.setPosition(absPosition);
		} else {
			double absPosition = Conversions.degreesToFalcon(
					pivotEncoder.getAbsolutePosition().getValueAsDouble() * 360.0 - kAbsOffsetOffset,
					Constants.Shooter.kPivotGearRatio);
			pivotLeader.setPosition(absPosition);
			pivotFollower.setPosition(absPosition);
		}
	}

	public void rawIndexer(double speed) {
		indexer.set(speed);
	}

	public void setShooterOutput(double speed) {
		shooterLeader.setControl(closedLoopVelocity.withVelocity(speed * Constants.kFalconFreeSpeedRPS));
	}

	public double getShooterVelocity() {
		return shooterLeader.getVelocity().getValueAsDouble();
	}

	// Gets the point of shooting relative to the origin of the robot.
	private Translation3d getShootingOrigin() {
		// I don't understand why a negative sign is needed; its probably because 3D
		// rotations need to pay attention to an AoR's direction.
		return GeometryUtils.rotatePointAboutPoint3D(Constants.Shooter.kDefaultShootingOrigin,
				Constants.Shooter.kPointOfRotation,
				new Rotation3d(0, Math.toRadians(-getPivotDegrees()), 0));
	}

	public double getPivotDegrees() {
		return pivotEncoder.getAbsolutePosition().getValue() * 360.0 - kAbsOffsetOffset;
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

		if (percentOutput < 0.0d && getPivotDegrees() < 3.0d) {
			if (!limitSwitch.get()) {
				pivotLeader.setControl(openLoop.withOutput(percentOutput * 12.0));
			}
		} else {
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
	}

	public boolean indexerIsIntaking() { // rename
		return currentState == IndexerState.IntakingNote;
	}

	public boolean indexerHasNote() {
		return hasNote;
	}

	@Override
	public void periodic() {
		/*-
		// Logic check does any of this make sense???
		// Also there should then be a function that can tell this thing that a note has
		// left and transitions the state to Idle.
		
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
			SmartDashboard.putString("INDEXER STATE", "Intaking Note");
			currentState = IndexerState.IntakingNote;
			hasNote = true;
		
		} else if (secondaryWasToF && !secondaryInstantToF && firstInstantToF) { //
			// If the second pass JUST Un-tripped, and the first pass is still tripped, we
			// are outtaking.
			SmartDashboard.putString("INDEXER STATE", "Outtaking Note");
			currentState = IndexerState.OuttakingNote;
			hasNote = false;
		} else if (firstWasToF && !firstInstantToF && secondaryInstantToF) { //
			// If the first pass JUST Un-tripped, and the second pass is still tripped, we
			// are shooting.
			SmartDashboard.putString("INDEXER STATE", "Shooting Note");
			currentState = IndexerState.ShootingNote;
			hasNote = false;
		}
		 */
		if (limitSwitch.get()) {
			// pivotEncoder.setPosition(kAbsOffsetOffset);
			// resetIntegratedToAbsolute(false);
		}
		// SmartDashboard.putNumber("i2c one",
		// initialToF.measure().distanceMillimeters);
		// SmartDashboard.putNumber("i2c two",
		// secondaryToF.measure().distanceMillimeters);
		SmartDashboard.putBoolean("LimitOfMyPatience", limitSwitch.get());
		SmartDashboard.putNumber("PivotIntegrated",
				Conversions.falconToDegrees(pivotLeader.getPosition().getValue(), Constants.Shooter.kPivotGearRatio));
		SmartDashboard.putNumber("PivotIntegrated2",
				Conversions.falconToDegrees(pivotFollower.getPosition().getValue(), Constants.Shooter.kPivotGearRatio));
		SmartDashboard.putNumber("PivotAbsolute",
				pivotEncoder.getAbsolutePosition().getValue() * 360.0 - kAbsOffsetOffset);
	}
}
