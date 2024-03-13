package com.redstorm509.alice2024.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
	private TalonFX leftClimbMotor = new TalonFX(17);
	private TalonFX rightClimbMotor = new TalonFX(18);

	private VoltageOut openLoopVoltage = new VoltageOut(0);

	private Solenoid leftSol = new Solenoid(PneumaticsModuleType.REVPH, 13); // changed to REVPH
	private Solenoid rightSol = new Solenoid(PneumaticsModuleType.REVPH, 15);

	private double bootRoll = 0.0d;

	public Climber(Pigeon2 pigeon) {
		TalonFXConfiguration conf = new TalonFXConfiguration();
		conf.CurrentLimits.SupplyCurrentLimitEnable = true;
		conf.CurrentLimits.SupplyCurrentLimit = 35.0;
		conf.MotorOutput.NeutralMode = NeutralModeValue.Coast;

		leftClimbMotor.getConfigurator().apply(conf);
		rightClimbMotor.getConfigurator().apply(conf);
		bootRoll = pigeon.getRoll().waitForUpdate(1).getValueAsDouble();
	}

	public double getBootRoll() {
		return bootRoll;
	}

	public void lockLeft() {
		if (leftSol.get()) {
			leftSol.set(false);
		}
	}

	public void lockRight() {
		if (rightSol.get()) {
			rightSol.set(false);
		}
	}

	public void unlockLeft() {
		leftSol.set(true);
	}

	public void unlockRight() {
		rightSol.set(true);
	}

	public void toggleLockLeft() {
		leftSol.toggle();
	}

	public void toggleLockRight() {
		rightSol.toggle();
	}

	public void leftClimb(double speed) {
		leftClimbMotor.setControl(openLoopVoltage.withOutput(speed * 12.0));
	}

	public void rightClimb(double speed) {
		rightClimbMotor.setControl(openLoopVoltage.withOutput(speed * 12.0));
	}

	public void climb(double speed) {
		leftClimb(speed);
		rightClimb(speed);
	}

	public double getLeftExtension() {
		return leftClimbMotor.getPosition().getValueAsDouble() * 0.0; // REPLACE ME rotations to milimeters conversion
	}

	public double getRightExtension() {
		return rightClimbMotor.getPosition().getValueAsDouble() * 0.0; // REPLACE ME rotations to milimeters conversion
	}

	@Override
	public void periodic() {
		SmartDashboard.putBoolean("LeftSol", leftSol.get());
		SmartDashboard.putBoolean("RightSol", rightSol.get());
	}
}