package com.redstorm509.alice2024.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;

import com.redstorm509.alice2024.Constants;
import com.redstorm509.alice2024.util.math.Conversions;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class SwerveModule {
	public int moduleNumber;

	// Motor Controllers for Drive/Steer and Steering Encoder
	private TalonFX steerMotor;
	private TalonFX driveMotor;
	private CANcoder steerEncoder;

	// Cached control requests for Pheonix 6
	private final PositionVoltage steerRequest = new PositionVoltage(0).withEnableFOC(false);
	private final VelocityVoltage closedLoopDriveRequest = new VelocityVoltage(0).withEnableFOC(false);
	private final VoltageOut openLoopDriveRequest = new VoltageOut(0).withEnableFOC(false);

	// The previously set steer angle.
	private double lastSteerAngleDeg;
	private SwerveModuleState lastSet = new SwerveModuleState();
	private SwerveModulePosition simulated = new SwerveModulePosition();

	// Construct a new Swerve Module using a preset Configuration
	public SwerveModule(Constants.SwerveModuleConfiguration configs) {
		this.moduleNumber = configs.moduleNumber();

		// Angle Encoder Config
		CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
		canCoderConfiguration.MagnetSensor.MagnetOffset = configs.steerEncoderOffset() / 360.0d;
		this.steerEncoder = new CANcoder(configs.steerEncoderId(), Constants.kCANIvore);
		this.steerEncoder.getConfigurator().apply(canCoderConfiguration);

		// Angle Motor Config
		TalonFXConfiguration steerMotorConfig = new TalonFXConfiguration();
		steerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		steerMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		steerMotorConfig.MotorOutput.DutyCycleNeutralDeadband = 0.02;
		steerMotorConfig.ClosedLoopGeneral.ContinuousWrap = true;
		steerMotorConfig.Feedback.FeedbackRemoteSensorID = configs.steerEncoderId();
		steerMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
		steerMotorConfig.Feedback.RotorToSensorRatio = Constants.MK4I.kAngleGearRatio;
		// The absolute encoder and mechanism are one-to-one, as the absolute
		// encoder is on the output shaft. However, the internal encoder and the
		// absoltue encoder do have a ratio equal to the steer gear ratio.

		// Phoenix ignores RotorToSensorRatio in the signals coming from the motor when
		// you configure it to use its internal sensor, but since we're configuring a
		// remote sensor this works. If we wanted to use the internal encoder, we would
		// instead use SensorToMechanismRatio.
		steerMotorConfig.Slot0.kP = Constants.kSteerAngleP;
		steerMotorConfig.Slot0.kI = Constants.kSteerAngleI;
		steerMotorConfig.Slot0.kD = Constants.kSteerAngleD;
		steerMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		steerMotorConfig.CurrentLimits.SupplyCurrentLimit = 35.0d;
		steerMotorConfig.CurrentLimits.SupplyCurrentThreshold = 45.0d;
		steerMotorConfig.CurrentLimits.SupplyTimeThreshold = 0.1;
		// Since we've just configured the steer motors to use data from the absolute
		// encoders, we don't need to divide the steer motor's position by the gear
		// ratio ourselves.

		this.steerMotor = new TalonFX(configs.steerMotorId(), Constants.kCANIvore);
		this.steerMotor.getConfigurator().apply(steerMotorConfig);

		// Drive Motor Config
		TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
		driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		driveMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		driveMotorConfig.MotorOutput.DutyCycleNeutralDeadband = 0.05;
		driveMotorConfig.Slot0.kP = Constants.kDriveVelocityP;
		driveMotorConfig.Slot0.kI = Constants.kDriveVelocityI;
		driveMotorConfig.Slot0.kD = Constants.kDriveVelocityD;
		driveMotorConfig.Slot0.kS = Constants.kDriveVelocityS;
		driveMotorConfig.Slot0.kV = Constants.kDriveVelocityV;
		driveMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		driveMotorConfig.CurrentLimits.SupplyCurrentLimit = 35.0d;
		driveMotorConfig.CurrentLimits.SupplyCurrentThreshold = 45.0d;
		driveMotorConfig.CurrentLimits.SupplyTimeThreshold = 0.1;

		this.driveMotor = new TalonFX(configs.driveMotorId(), Constants.kCANIvore);
		this.driveMotor.getConfigurator().apply(driveMotorConfig);

		this.driveMotor.setPosition(0);

		// We wait 1.0s for a sensor reading on startup to ensure its validity.
		this.lastSteerAngleDeg = steerMotor.getPosition().waitForUpdate(1.0).getValueAsDouble() * 360.0;
	}

	public Rotation2d getAngle() {
		return Rotation2d.fromRotations(steerMotor.getPosition().getValue());
	}

	public SwerveModulePosition getPosition() {
		if (RobotBase.isSimulation()) {
			return simulated;
		}
		double driveRot = BaseStatusSignal.getLatencyCompensatedValue(driveMotor.getPosition(),
				driveMotor.getVelocity());
		double steerRot = BaseStatusSignal.getLatencyCompensatedValue(steerMotor.getPosition(),
				steerMotor.getVelocity());

		// Accounts for coupling; rotation of the angle motor causing the actual drive
		// wheel to rotate slightly.
		double drivePosition = driveRot - steerRot * Constants.MK4I.kCouplingRatio;
		return new SwerveModulePosition(
				Conversions.falconToMeters(
						drivePosition,
						Constants.MK4I.kWheelCircumference,
						Constants.MK4I.kDriveGearRatio),
				Rotation2d.fromRotations(steerRot));
	}

	public SwerveModuleState getState() {
		if (RobotBase.isSimulation()) {
			return new SwerveModuleState(lastSet.speedMetersPerSecond, simulated.angle);
		}
		return new SwerveModuleState(
				Conversions.falconToMPS(
						driveMotor.getVelocity().getValue(),
						Constants.MK4I.kWheelCircumference,
						Constants.MK4I.kDriveGearRatio),
				getAngle());
	}

	public void setDesiredState(SwerveModuleState desiredState, boolean closedLoop) {
		lastSet = desiredState;
		// Ensures that the module takes the optimal path towards the target angle,
		// limiting rotation to only 90 degrees at a time.
		desiredState = SwerveModuleState.optimize(desiredState, getAngle());

		double optimalTargetAngle = desiredState.angle.getDegrees();

		if (Math.abs(optimalTargetAngle - lastSteerAngleDeg) < Constants.kMaxAngularVelocity * 0.01) {
			optimalTargetAngle = lastSteerAngleDeg;
		}

		lastSteerAngleDeg = optimalTargetAngle;
		steerMotor.setControl(steerRequest.withPosition(optimalTargetAngle / 360.0));

		// This is the new stuff CTRE added in their Swerve API.
		// The cosineScalar stuff is supposed to combat the skew that occurs when you
		// change direction.
		// The driveRateBackOut stuff accounts for coupling; rotation of the angle motor
		// causing the actual drive wheel to rotate slightly.
		double steerErrorRad = Math.toRadians(optimalTargetAngle - getAngle().getDegrees());
		double cosineScalar = Math.cos(steerErrorRad);
		if (cosineScalar < 0.0d) {
			cosineScalar = 0.0d;
		}
		// double driveRateBackOut = steerMotor.getVelocity().getValue() *
		// Constants.MK4I.kCouplingRatio;

		double targetVelocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond,
				Constants.MK4I.kWheelCircumference, Constants.MK4I.kDriveGearRatio) * cosineScalar;

		if (closedLoop) {
			driveMotor.setControl(closedLoopDriveRequest.withVelocity(targetVelocity));
		} else {
			double voltage = Conversions.falconToMPS(targetVelocity, Constants.MK4I.kWheelCircumference,
					Constants.MK4I.kDriveGearRatio) / Constants.kMaxSpeed * 12.0;
			driveMotor.setControl(openLoopDriveRequest.withOutput(voltage));
		}
	}

	public double getSupplyCurrent() {
		return driveMotor.getSupplyCurrent().getValueAsDouble() + steerMotor.getSupplyCurrent().getValueAsDouble();
	}

	public void simPeriodic() {
		double error = (lastSet.angle.getRotations() - simulated.angle.getRotations()) / Constants.MK4I.kAngleGearRatio;
		double rotationsPerSecond = Constants.kFalconFreeSpeedRPS * error * Constants.kSteerAngleP / 12.0;

		simulated.angle = Rotation2d.fromRotations(simulated.angle.getRotations() + rotationsPerSecond * 0.02);
		simulated.distanceMeters += lastSet.speedMetersPerSecond * 0.02;
	}

	public void resetSimState() {
		simulated.angle = Rotation2d.fromDegrees(0);
		simulated.distanceMeters = 0.0d;
	}
}