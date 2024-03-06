package com.redstorm509.alice2024.subsystems;

import com.redstorm509.alice2024.Constants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

	private TalonFX shooterLeader = new TalonFX(15); // Labelled SHOOTERL
	private TalonFX shooterFollower = new TalonFX(16); // Labelled SHOOTERR

	private VelocityVoltage closedLoopVelocity = new VelocityVoltage(0).withEnableFOC(false);

	// TODO: Define coordinate space!
	public Shooter() {
		TalonFXConfiguration shootConf = new TalonFXConfiguration();
		shootConf.CurrentLimits.SupplyCurrentLimitEnable = true;
		shootConf.CurrentLimits.SupplyCurrentLimit = 35.0;
		shootConf.Slot0.kP = Constants.Shooter.kFlyWheelP;
		shootConf.Slot0.kI = Constants.Shooter.kFlyWheelI;
		shootConf.Slot0.kD = Constants.Shooter.kFlyWheelD;
		shootConf.Slot0.kS = Constants.Shooter.kFlyWheelS;
		shootConf.Slot0.kV = Constants.Shooter.kFlyWheelV;
		shootConf.Slot0.kA = Constants.Shooter.kFlyWheelA;
		shootConf.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		shooterLeader.getConfigurator().apply(shootConf);
		shooterFollower.getConfigurator().apply(shootConf);
	}

	public void setShooterVelocity(double speed) {
		shooterLeader.setControl(closedLoopVelocity.withVelocity(speed));
	}

	public double getShooterVelocity() {
		return shooterLeader.getVelocity().getValueAsDouble();
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Shooter Velocity (rot/s)", shooterLeader.getVelocity().getValue());
		SmartDashboard.putNumber("Shooter Velocity (m/s)",
				Constants.Shooter.kFlyWheelCircumference * shooterLeader.getVelocity().getValue());
	}
}
