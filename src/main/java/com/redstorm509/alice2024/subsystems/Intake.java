package com.redstorm509.alice2024.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.redstorm509.alice2024.Constants;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
	public final TalonFX intakeMotor = new TalonFX(12);
	private final CANSparkMax preCompressorMotors = new CANSparkMax(10, MotorType.kBrushed);
	private final CANSparkMax intermediateStage = new CANSparkMax(16, MotorType.kBrushless);
	private VoltageOut openLoopVoltage = new VoltageOut(0);
	private double preCompressorSpeed = 0.0d;

	public Intake() {
		TalonFXConfiguration conf = new TalonFXConfiguration();
		conf.CurrentLimits.StatorCurrentLimitEnable = true;
		conf.CurrentLimits.StatorCurrentLimit = 35.0;
		intakeMotor.getConfigurator().apply(conf);

		preCompressorMotors.setSmartCurrentLimit(18);
		preCompressorMotors.setIdleMode(IdleMode.kCoast);
		preCompressorMotors.burnFlash();

		// intermediateStage.setSmartCurrentLimit(17);
		// intermediateStage.setIdleMode(IdleMode.kCoast);
		// intermediateStage.burnFlash();
	}

	@Override
	public void periodic() {
		intakeMotor.setControl(openLoopVoltage);
		preCompressorMotors.set(preCompressorSpeed);
	}

	@Override
	public void simulationPeriodic() {
		// This method will be called once per scheduler run during simulation
	}

	public void intake(boolean inwards) {
		if (inwards) {
			openLoopVoltage.Output = (-Constants.Intake.kIntakeSpinSpeed * 12);
		} else {
			openLoopVoltage.Output = (Constants.Intake.kIntakeSpinSpeed * 12);
		}
		preCompressorSpeed = Constants.PreCompressor.kPreCompressorSpinSpeed;
	}

	public void stop() {
		openLoopVoltage.Output = 0;
		preCompressorSpeed = 0.0d;
	}
}
