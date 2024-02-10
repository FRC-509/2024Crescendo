package com.redstorm509.alice2024.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
	// definitely not working

	private TalonFX leftClimbMotor = new TalonFX(0); // ID This
	private TalonFX rightClimbMotor = new TalonFX(0); // ID This

	private VoltageOut openLoopVoltage = new VoltageOut(0);

	private DigitalInput leftLimitSwitch = new DigitalInput(0); // ID This
	private DigitalInput rightLimitSwitch = new DigitalInput(0); // ID This

	public Climber() {
		TalonFXConfiguration conf = new TalonFXConfiguration();
		conf.CurrentLimits.StatorCurrentLimitEnable = true;
		conf.CurrentLimits.StatorCurrentLimit = 35.0;

		leftClimbMotor.getConfigurator().apply(conf);
		rightClimbMotor.getConfigurator().apply(conf);
	}

	public void syncedClimb(double extensionLength) {
		leftClimb(extensionLength);
		rightClimb(extensionLength);
	}

	public void leftClimb(double extensionLength) {
	}

	public void rightClimb(double extensionLength) {
	}

}
