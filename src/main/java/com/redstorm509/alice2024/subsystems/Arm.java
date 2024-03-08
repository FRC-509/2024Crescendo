package com.redstorm509.alice2024.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.redstorm509.alice2024.Constants;
import com.redstorm509.alice2024.util.math.Conversions;
import com.redstorm509.stormkit.math.PositionTarget;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
	private static final double kAbsOffsetOffset = 0.0;

	private TalonFX pivotLeader = new TalonFX(13); // Labelled PIVOTL
	private TalonFX pivotFollower = new TalonFX(14); // Labelled PIVOTR
	private CANcoder pivotEncoder = new CANcoder(17);
	private DigitalInput limitSwitch = new DigitalInput(6);

	private VoltageOut openLoop = new VoltageOut(0).withEnableFOC(false);
	private PositionVoltage closedLoopPosition = new PositionVoltage(0).withEnableFOC(false);

	private PositionTarget pivotTarget;

	public Arm() {
		TalonFXConfiguration pivotConf = new TalonFXConfiguration();
		pivotConf.CurrentLimits.SupplyCurrentLimitEnable = true;
		pivotConf.CurrentLimits.SupplyCurrentLimit = 35.0;
		pivotConf.Slot0.kP = Constants.Arm.kPivotP;
		pivotConf.Slot0.kI = Constants.Arm.kPivotI;
		pivotConf.Slot0.kD = Constants.Arm.kPivotD;
		pivotConf.MotorOutput.NeutralMode = NeutralModeValue.Coast;

		pivotLeader.getConfigurator().apply(pivotConf);
		pivotFollower.getConfigurator().apply(pivotConf);

		CANcoderConfiguration pivotEncoderConf = new CANcoderConfiguration();
		pivotEncoderConf.MagnetSensor.MagnetOffset = (Constants.Arm.kPivotMagnetOffset + kAbsOffsetOffset) / 360.0;
		pivotEncoderConf.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
		pivotEncoderConf.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
		pivotEncoder.getConfigurator().apply(pivotEncoderConf);

		pivotFollower.setControl(new Follower(pivotLeader.getDeviceID(), true));

		resetIntegratedToAbsolute(true);

		if (pivotEncoder.getPosition().getValueAsDouble() * 360.0d >= 350.0d || limitSwitch.get()) {
			pivotEncoder.setPosition(0);
			pivotLeader.setPosition(0);
			pivotFollower.setPosition(0);
		}
		double angle = pivotEncoder.getPosition().getValueAsDouble() * 360.0d;
		pivotTarget = new PositionTarget(angle, Constants.Arm.kMinPivot, Constants.Arm.kMaxPivot,
				Constants.Arm.kMaxPivotSpeed);
	}

	public void resetIntegratedToAbsolute(boolean waitForUpdate) {
		if (waitForUpdate) {
			double absPosition = Conversions.degreesToFalcon(
					pivotEncoder.getPosition().waitForUpdate(1).getValueAsDouble() * 360.0 - kAbsOffsetOffset,
					Constants.Arm.kPivotGearRatio);
			pivotLeader.setPosition(absPosition);
			pivotFollower.setPosition(absPosition);
		} else {
			double absPosition = Conversions.degreesToFalcon(
					pivotEncoder.getPosition().getValueAsDouble() * 360.0 - kAbsOffsetOffset,
					Constants.Arm.kPivotGearRatio);
			pivotLeader.setPosition(absPosition);
			pivotFollower.setPosition(absPosition);
		}
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
		double ticks = Conversions.degreesToFalcon(target, Constants.Arm.kPivotGearRatio);

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

	@Override
	public void periodic() {
		// DIO channels default to high in sim so we dont run this code if we're in sim.
		if (!RobotBase.isSimulation() && limitSwitch.get()) {
			pivotEncoder.setPosition(kAbsOffsetOffset);
			resetIntegratedToAbsolute(false);
		}
		SmartDashboard.putBoolean("PivotLimitSwitch", limitSwitch.get());
		SmartDashboard.putNumber("PivotIntegratedRaw", pivotLeader.getPosition().getValueAsDouble());
		SmartDashboard.putNumber("PivotLIntegrated",
				Conversions.falconToDegrees(pivotLeader.getPosition().getValue(), Constants.Arm.kPivotGearRatio));
		SmartDashboard.putNumber("PivotFIntegrated",
				Conversions.falconToDegrees(pivotFollower.getPosition().getValue(), Constants.Arm.kPivotGearRatio));
		SmartDashboard.putNumber("PivotEncoder (Relative)",
				pivotEncoder.getAbsolutePosition().getValue() * 360.0 - kAbsOffsetOffset);
		SmartDashboard.putNumber("PivotEncoder (Absolute)",
				pivotEncoder.getAbsolutePosition().getValue() * 360.0 - kAbsOffsetOffset);
	}
}
