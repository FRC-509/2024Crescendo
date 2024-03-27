package com.redstorm509.alice2024.subsystems;

import com.redstorm509.alice2024.Constants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

	public TalonFX shooterLeader = new TalonFX(15); // Labelled SHOOTERL
	private TalonFX shooterFollower = new TalonFX(16); // Labelled SHOOTERR

	private VelocityVoltage closedLoopVelocity = new VelocityVoltage(0).withEnableFOC(false);

	// TODO: Define coordinate space!
	public Shooter() {
		TalonFXConfiguration shootConf = new TalonFXConfiguration();
		shootConf.CurrentLimits.SupplyCurrentLimitEnable = true;
		shootConf.CurrentLimits.SupplyCurrentLimit = 40.0;
		shootConf.Slot0.kP = Constants.Shooter.kFlyWheelP;
		shootConf.Slot0.kI = Constants.Shooter.kFlyWheelI;
		shootConf.Slot0.kD = Constants.Shooter.kFlyWheelD;
		shootConf.Slot0.kS = Constants.Shooter.kFlyWheelS;
		shootConf.Slot0.kV = Constants.Shooter.kFlyWheelV;
		shootConf.Slot0.kA = Constants.Shooter.kFlyWheelA;
		shootConf.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		shooterLeader.getConfigurator().apply(shootConf);
		shooterFollower.getConfigurator().apply(shootConf);
		// shooterFollower.setInverted(true);
		shooterFollower.setControl(new Follower(shooterLeader.getDeviceID(), true));
	}

	public void setShooterVelocity(double speed) {
		shooterLeader.setControl(closedLoopVelocity.withVelocity(speed));
		// shooterFollower.setControl(closedLoopVelocity.withVelocity(speed - 1.5));

	}

	public double getShooterVelocity() {
		return shooterLeader.getVelocity().getValueAsDouble();
	}

	public boolean isAtShooterVelocity() {
		return Math.abs(Math.abs(getShooterVelocity()) - Constants.Shooter.kTargetSpeed) <= 2.0d;
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Shooter Velocity (rot/s)", shooterLeader.getVelocity().getValue());
	}

	public Command startShooting() {
		return Commands.runOnce(() -> setShooterVelocity(-Constants.Shooter.kTargetSpeed), this);
	}

	public Command stopShooting() {
		return Commands.runOnce(() -> shooterLeader.setControl(new VoltageOut(0)), this);
	}
}
