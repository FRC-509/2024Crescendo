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

	public Intake() {
		TalonFXConfiguration conf = new TalonFXConfiguration();
		conf.CurrentLimits.SupplyCurrentLimitEnable = true;
		conf.CurrentLimits.SupplyCurrentLimit = 35.0;

		intakeMotor.getConfigurator().apply(conf);

		preCompressorMotors.setSmartCurrentLimit(30);
		preCompressorMotors.setIdleMode(IdleMode.kCoast);
		preCompressorMotors.setInverted(true);
		preCompressorMotors.burnFlash();

		intermediateStage.setSmartCurrentLimit(15);
		intermediateStage.setIdleMode(IdleMode.kCoast);
		intermediateStage.burnFlash();
	}

	public void intake(boolean inwards) {
		if (inwards) {
			intakeMotor.setControl(openLoopVoltage.withOutput(-Constants.Intake.kIntakeSpinSpeed * 12));
			preCompressorMotors.set(-Constants.Intake.kPreCompressorSpinSpeed);
			intermediateStage.set(-Constants.Intake.kIntermediateStageSpinSpeed);
		} else {
			intakeMotor.setControl(openLoopVoltage.withOutput(Constants.Intake.kIntakeSpinSpeed * 12));
			preCompressorMotors.set(Constants.Intake.kPreCompressorSpinSpeed);
			intermediateStage.set(Constants.Intake.kIntermediateStageSpinSpeed);
		}
	}

	public void stop() {
		intakeMotor.setControl(openLoopVoltage.withOutput(0));
		preCompressorMotors.set(0);
		intermediateStage.set(0);
	}
}
