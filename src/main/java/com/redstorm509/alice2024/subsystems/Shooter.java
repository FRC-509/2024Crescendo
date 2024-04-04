package com.redstorm509.alice2024.subsystems;

import com.redstorm509.alice2024.Constants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
	private double kMaxShooterAccel = 230.0d;
	private TalonFX shooterLeader = new TalonFX(16); // Labelled SHOOTERT
	private TalonFX shooterFollower = new TalonFX(15); // Labelled SHOOTERB
	private SlewRateLimiter rateLimiter = new SlewRateLimiter(kMaxShooterAccel);
	private double goalVelocity = 0.0d;

	private VelocityVoltage closedLoopVelocity = new VelocityVoltage(0).withEnableFOC(false);
	private VoltageOut openLoop = new VoltageOut(0);

	// TODO: Define coordinate space!
	public Shooter() {
		TalonFXConfiguration shootConf = new TalonFXConfiguration();
		shootConf.CurrentLimits.SupplyCurrentLimitEnable = true;
		shootConf.CurrentLimits.SupplyCurrentLimit = 30.0;
		shootConf.Slot0.kP = Constants.Shooter.kFlyWheelP;
		shootConf.Slot0.kI = Constants.Shooter.kFlyWheelI;
		shootConf.Slot0.kD = Constants.Shooter.kFlyWheelD;
		shootConf.Slot0.kS = Constants.Shooter.kFlyWheelS;
		shootConf.Slot0.kV = Constants.Shooter.kFlyWheelV;
		shootConf.Slot0.kA = Constants.Shooter.kFlyWheelA;
		shootConf.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		shooterLeader.getConfigurator().apply(shootConf);
		shooterFollower.getConfigurator().apply(shootConf);

		shooterFollower.setControl(new Follower(shooterLeader.getDeviceID(), false));
	}

	public void setShooterVelocity(double speed) {
		goalVelocity = speed;
		// rateLimiter.calculate(speed);
		// shooterLeader.setControl(closedLoopVelocity.withVelocity(speed));
	}

	public double getGoalVelocity() {
		return goalVelocity;
	}

	public double getShooterVelocity() {
		return shooterLeader.getVelocity().getValueAsDouble();
	}

	public boolean isAtShooterVelocity() {
		return Math.abs(getShooterVelocity()) >= Math.abs(goalVelocity) - 7.0;
	}

	public void setToDefaultShootSpeed() {
		goalVelocity = -Constants.Shooter.kDefaultShootSpeed;
	}

	@Override
	public void periodic() {
		if (goalVelocity == 0.0d) {
			rateLimiter.reset(0.0d);
			shooterLeader.setControl(openLoop.withOutput(0));
		} else {
			double rateLimitedSetpoint = rateLimiter.calculate(goalVelocity);
			shooterLeader.setControl(closedLoopVelocity.withVelocity(rateLimitedSetpoint));
		}
		SmartDashboard.putNumber("Shooter Supply Current",
				shooterLeader.getSupplyCurrent().getValueAsDouble()
						+ shooterFollower.getSupplyCurrent().getValueAsDouble());
		SmartDashboard.putNumber("Shooter Velocity (rps)", Math.abs(shooterLeader.getVelocity().getValueAsDouble()));
	}

	public Command startShooting() {
		return Commands.runOnce(() -> setShooterVelocity(Constants.Shooter.kTargetSpeed), this);
	}

	public Command stopShooting() {
		return Commands.runOnce(() -> setShooterVelocity(0), this);
	}
}
