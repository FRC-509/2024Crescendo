package com.redstorm509.alice2024.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PreCompressor extends SubsystemBase {

	private final CANSparkMax leftMotor = new CANSparkMax(0, MotorType.kBrushless);
	private final CANSparkMax rightMotor = new CANSparkMax(0, MotorType.kBrushless);

	public PreCompressor() {
		rightMotor.setInverted(true);
		rightMotor.setSmartCurrentLimit(18);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	@Override
	public void simulationPeriodic() {
		// This method will be called once per scheduler run during simulation
	}

	public void spin(double speed) {
		rightMotor.set(speed);
		leftMotor.set(speed);
	}
}
