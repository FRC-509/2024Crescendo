package com.redstorm509.alice2024.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
	private TalonFX leftClimbMotor = new TalonFX(17);
	private TalonFX rightClimbMotor = new TalonFX(18);

	private VoltageOut openLoopVoltage = new VoltageOut(0);
	private Solenoid leftSol = new Solenoid(PneumaticsModuleType.CTREPCM, 6);
	private Solenoid rightSol = new Solenoid(PneumaticsModuleType.CTREPCM, 7);

	public Climber() {
		TalonFXConfiguration conf = new TalonFXConfiguration();
		conf.CurrentLimits.SupplyCurrentLimitEnable = true;
		conf.CurrentLimits.SupplyCurrentLimit = 35.0;
		conf.MotorOutput.NeutralMode = NeutralModeValue.Brake;

		leftClimbMotor.getConfigurator().apply(conf);
		rightClimbMotor.getConfigurator().apply(conf);
	}

	public void leftClimb(double speed) {
		if (Math.abs(speed) >= 0.1) {
			if (!leftSol.get()) {
				leftSol.set(true);
			}
			leftClimbMotor.setControl(openLoopVoltage.withOutput(speed * 12.0));
		} else {
			leftClimbMotor.setControl(openLoopVoltage.withOutput(0));
			if (leftSol.get()) {
				leftSol.set(false);
			}
		}
	}

	public void rightClimb(double speed) {
		if (Math.abs(speed) >= 0.1) {
			if (!rightSol.get()) {
				rightSol.set(true);
			}
			rightClimbMotor.setControl(openLoopVoltage.withOutput(speed * 12.0));
		} else {
			rightClimbMotor.setControl(openLoopVoltage.withOutput(0));
			if (rightSol.get()) {
				rightSol.set(false);
			}
		}
	}
}
