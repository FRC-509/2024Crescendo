package com.redstorm509.alice2024.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PreCompressor extends SubsystemBase {

	private final CANSparkMax motor = new CANSparkMax(10, MotorType.kBrushed);

	public PreCompressor() {
		// motor.setInverted(true);
		motor.setSmartCurrentLimit(18);
		motor.setIdleMode(IdleMode.kBrake);
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
		motor.set(speed);
	}
}
