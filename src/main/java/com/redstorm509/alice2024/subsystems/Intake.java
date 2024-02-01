package com.redstorm509.alice2024.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.redstorm509.alice2024.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
	public TalonFX intakeMotor = new TalonFX(12);
	private VoltageOut openLoopVoltage = new VoltageOut(0);

	public Intake() {
		TalonFXConfiguration conf = new TalonFXConfiguration();
		conf.CurrentLimits.StatorCurrentLimitEnable = true;
		conf.CurrentLimits.StatorCurrentLimit = 35.0;
		intakeMotor.getConfigurator().apply(conf);
	}

	@Override
	public void periodic() {
		intakeMotor.setControl(openLoopVoltage);
	}

	@Override
	public void simulationPeriodic() {
		// This method will be called once per scheduler run during simulation
	}

	public void intake(boolean inwards) {
		if (inwards) {
			openLoopVoltage = openLoopVoltage.withOutput(-Constants.Intake.kIntakeSpinSpeed * 12);
		} else {
			openLoopVoltage = openLoopVoltage.withOutput(Constants.Intake.kIntakeSpinSpeed * 12);
		}
	}

	public void stop() {
		openLoopVoltage = openLoopVoltage.withOutput(0);
	}
}
