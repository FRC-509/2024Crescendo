package com.redstorm509.alice2024.subsystems;

import com.redstorm509.alice2024.Constants;
import com.redstorm509.alice2024.util.math.Conversions;
import com.redstorm509.stormkit.math.PositionTarget;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
	public enum IndexerState {
		HasNote,
		Noteless,
		NoteTooShooter,
		NoteTooShooterExtreme,
		NoteTooIntake,
		NoteTooIntakeExtreme,
	}

	private static final double kAbsOffsetOffset = 0.0;

	private TalonFX shooterLeader = new TalonFX(15); // Labelled SHOOTERL
	private TalonFX shooterFollower = new TalonFX(16); // Labelled SHOOTERR

	private CANSparkMax indexer = new CANSparkMax(12, MotorType.kBrushed);
	private DigitalInput shooterBB = new DigitalInput(1); // CHANGE TO REAL PORTS
	private DigitalInput indexerBB = new DigitalInput(2);
	private DigitalInput imStageBB = new DigitalInput(3);

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

		if (pivotEncoder.getPosition().getValueAsDouble() * 360.0d >= 350.0d) {
			pivotEncoder.setPosition(0);
			pivotLeader.setPosition(0);
			pivotFollower.setPosition(0);
		}
		double angle = pivotEncoder.getPosition().getValueAsDouble() * 360.0d;
		pivotTarget = new PositionTarget(angle, Constants.Shooter.kMinPivot, Constants.Shooter.kMaxPivot,
				Constants.Shooter.kMaxPivotSpeed);
	}

	public void resetIntegratedToAbsolute(boolean waitForUpdate) {
		if (waitForUpdate) {
			double absPosition = Conversions.degreesToFalcon(
					pivotEncoder.getPosition().waitForUpdate(1).getValueAsDouble() * 360.0 - kAbsOffsetOffset,
					Constants.Shooter.kPivotGearRatio);
			pivotLeader.setPosition(absPosition);
			pivotFollower.setPosition(absPosition);
		} else {
			double absPosition = Conversions.degreesToFalcon(
					pivotEncoder.getPosition().getValueAsDouble() * 360.0 - kAbsOffsetOffset,
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

	public double getPivotDegrees() {
		return pivotEncoder.getPosition().getValue() * 360.0 - kAbsOffsetOffset;
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

		// double previous = pivotTarget.getTarget();
		pivotTarget.update(percentOutput);

		if (percentOutput <= 0.0d && getPivotDegrees() < 3.0d) {
			if (!limitSwitch.get()) {
				pivotLeader.setControl(openLoop.withOutput(percentOutput));
			}
		} else if (percentOutput <= 0 && getPivotDegrees() < 0) {
			pivotLeader.setControl(openLoop.withOutput(percentOutput * 0.075));
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

	public boolean hasNote() {
		return indexingNoteState() == IndexerState.HasNote;
	}

	public IndexerState indexingNoteState() {
		if (indexerBB.get() && shooterBB.get() && !imStageBB.get()) {
			// Note is where we want it to be
			return IndexerState.HasNote;
		} else if (indexerBB.get() && !shooterBB.get() && !imStageBB.get()) {
			// Note is too far out shooter side
			return IndexerState.NoteTooShooter;
		} else if (indexerBB.get() && !shooterBB.get() && imStageBB.get()) {
			// Note is too far out intermediate stage side
			return IndexerState.NoteTooIntake;
		} else if (indexerBB.get() && !shooterBB.get() && !imStageBB.get()) {
			// Note is inside of indexer, but not far enough
			return IndexerState.NoteTooIntake;
		} else if (!indexerBB.get() && shooterBB.get() && !imStageBB.get()) {
			// Note is way too sticking out shooter
			return IndexerState.NoteTooShooterExtreme;
		} else if (!indexerBB.get() && !shooterBB.get() && imStageBB.get()) {
			// Note is way too sticking out shooter
			return IndexerState.NoteTooIntakeExtreme;
		} else {
			// Does not have a note, or is invalid state
			return IndexerState.Noteless;
		}
	}

	@Override
	public void periodic() {
		// DIO channels default to high in sim so we dont run this code if we're in sim.
		if (!RobotBase.isSimulation() && limitSwitch.get()) {
			// pivotEncoder.setPosition(kAbsOffsetOffset);
			resetIntegratedToAbsolute(false);
		}
		SmartDashboard.putBoolean("PivotLimitSwitch", limitSwitch.get());
		SmartDashboard.putNumber("PivotLIntegrated",
				Conversions.falconToDegrees(pivotLeader.getPosition().getValue(), Constants.Shooter.kPivotGearRatio));
		SmartDashboard.putNumber("PivotFIntegrated",
				Conversions.falconToDegrees(pivotFollower.getPosition().getValue(), Constants.Shooter.kPivotGearRatio));
		SmartDashboard.putNumber("PivotAbsolute", pivotEncoder.getPosition().getValue() * 360.0 - kAbsOffsetOffset);
		SmartDashboard.putNumber("ShooterVelocity (r/s)", shooterLeader.getVelocity().getValue());
		SmartDashboard.putNumber("ShooterVelocity (m/s)",
				Constants.Shooter.kFlyWheelCircumference * shooterLeader.getVelocity().getValue());

	}
}
