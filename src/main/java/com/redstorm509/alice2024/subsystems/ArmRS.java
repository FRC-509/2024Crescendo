package com.redstorm509.alice2024.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.redstorm509.alice2024.Constants;
import com.redstorm509.stormkit.math.PositionTarget;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmRS extends SubsystemBase {
	private TalonFX pivotLeader = new TalonFX(13); // Labelled PIVOTL
	private TalonFX pivotFollower = new TalonFX(14); // Labelled PIVOTR
	private CANcoder pivotEncoder = new CANcoder(17);
	private DigitalInput limitSwitch = new DigitalInput(6);
	private boolean wasLimitSwitchTripped = false;

	private VoltageOut openLoop = new VoltageOut(0).withEnableFOC(false);
	private PositionVoltage closedLoopPosition = new PositionVoltage(0).withEnableFOC(false);
	private PositionTarget pivotTarget;

	public ArmRS() {
		TalonFXConfiguration pivotConf = new TalonFXConfiguration();
		pivotConf.CurrentLimits.SupplyCurrentLimitEnable = true;
		pivotConf.CurrentLimits.SupplyCurrentLimit = 35.0;
		pivotConf.Slot0.kP = Constants.Arm.kPivotRSP;
		pivotConf.Slot0.kI = Constants.Arm.kPivotRSI;
		pivotConf.Slot0.kD = Constants.Arm.kPivotRSD;
		pivotConf.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		pivotConf.MotorOutput.DutyCycleNeutralDeadband = 0.02;

		pivotConf.Feedback.FeedbackRemoteSensorID = pivotEncoder.getDeviceID();
		pivotConf.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
		pivotConf.Feedback.RotorToSensorRatio = Constants.Arm.kPivotGearRatio;
		// The absolute encoder and mechanism are one-to-one, as the absolute
		// encoder is on the output shaft. However, the internal encoder and the
		// absoltue encoder do have a ratio equal to the steer gear ratio.

		pivotLeader.getConfigurator().apply(pivotConf);
		pivotFollower.getConfigurator().apply(pivotConf);

		CANcoderConfiguration pivotEncoderConf = new CANcoderConfiguration();
		pivotEncoderConf.MagnetSensor.MagnetOffset = (Constants.Arm.kPivotMagnetOffset) / 360.0;
		pivotEncoderConf.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
		pivotEncoderConf.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
		pivotEncoder.getConfigurator().apply(pivotEncoderConf);

		pivotFollower.setControl(new Follower(pivotLeader.getDeviceID(), true));

		pivotLeader.setPosition(Constants.Arm.kMinPivot / 360.0);

		double angle = pivotLeader.getPosition().waitForUpdate(1.0).getValueAsDouble() * 360.0d;
		pivotTarget = new PositionTarget(angle, Constants.Arm.kMinPivot, Constants.Arm.kMaxPivot,
				Constants.Arm.kMaxPivotSpeed);
	}

	public double getPivotDegrees() {
		return pivotLeader.getPosition().getValue() * 360.0;
	}

	public void setPivotDegrees(double targetDegrees) {
		double delta = (targetDegrees - getPivotDegrees()) % 360;

		if (delta > 180.0d) {
			delta -= 360.0d;
		} else if (delta < -180.0d) {
			delta += 360.0d;
		}

		double target = getPivotDegrees() + delta;
		double ticks = target / 360.0d;

		pivotLeader.setControl(closedLoopPosition.withPosition(ticks));

		pivotTarget.setTarget(target);
	}

	public void setPivotOpenLoop(double percentOutput) {
		pivotLeader.setControl(openLoop.withOutput(percentOutput * 0.25));
	}

	public void setPivotOutput(double percentOutput) {
		// double previous = pivotTarget.getTarget();
		pivotTarget.update(percentOutput);

		if (percentOutput <= 0.0d && getPivotDegrees() < (Constants.Arm.kMinPivot + 5.0d)) {
			if (!limitSwitch.get()) {
				pivotLeader.setControl(openLoop.withOutput(percentOutput));
			}
		} else if (percentOutput <= 0 && getPivotDegrees() < Constants.Arm.kMinPivot) {
			pivotLeader.setControl(openLoop.withOutput(percentOutput));
		} else {
			/*-
			This is where we will add softstops
			if (percentOutput < 0.0d && !isValidState(pivotTarget.getTarget(), getArmLength())) {
				pivotTarget.setTarget(previous);
			}
			*/

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
		if (!RobotBase.isSimulation() && !wasLimitSwitchTripped && limitSwitch.get()) {
			pivotEncoder.setPosition(Constants.Arm.kMinPivot / 360.0);
		}
		wasLimitSwitchTripped = limitSwitch.get();
		SmartDashboard.putBoolean("PivotLimitSwitch", limitSwitch.get());
		SmartDashboard.putNumber("PivotRaw", pivotLeader.getPosition().getValue());
		SmartDashboard.putNumber("PivotL", pivotLeader.getPosition().getValue() * 360.0d);
		SmartDashboard.putNumber("PivotF", pivotFollower.getPosition().getValue() * 360.0d);
	}
}
