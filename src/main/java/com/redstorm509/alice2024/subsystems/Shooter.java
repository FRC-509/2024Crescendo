package com.redstorm509.alice2024.subsystems;

import com.redstorm509.alice2024.Constants;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
	private TalonFX shootMotor = new TalonFX(0);
	private TalonFX leftRotationMotor = new TalonFX(0);
	private TalonFX rightRotationMotor = new TalonFX(0);
	private CANcoder rotationEncoder = new CANcoder(0);
	private PIDController rotationPID = new PIDController(0.0, 0.0, 0.0);

	private VoltageOut voltageControlReq = new VoltageOut(0);

	public Shooter() {
		TalonFXConfiguration conf = new TalonFXConfiguration();
		conf.CurrentLimits.StatorCurrentLimitEnable = true;
		conf.CurrentLimits.StatorCurrentLimit = 35.0;
		shootMotor.getConfigurator().apply(conf);

	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	@Override
	public void simulationPeriodic() {
		// This method will be called once per scheduler run during simulation
	}

	public void shoot() {
		// Math for shooter tbd
	}

	public double getPivotDegrees() {
		return 0.0;
	}

	public void setPivotDegrees(double degrees) {
	}

	public void setPivotOutput() {
	}
}
