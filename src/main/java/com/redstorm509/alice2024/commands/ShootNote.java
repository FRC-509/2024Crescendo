package com.redstorm509.alice2024.commands;

import java.util.function.DoubleSupplier;

import com.redstorm509.alice2024.Constants;
import com.redstorm509.alice2024.subsystems.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootNote extends Command {
	private Shooter shooter;
	private double speedRps;
	private boolean hasReachedVel = false;
	private Timer timer;

	public ShootNote(Shooter shooter, double speedRotationsPerSecond) {
		this.shooter = shooter;
		this.speedRps = speedRotationsPerSecond;
		this.timer = new Timer();
		addRequirements(shooter);
	}

	@Override
	public void initialize() {
		timer.reset();
		timer.start();
		hasReachedVel = false;
		shooter.rawIndexer(0);
		shooter.setShooterOutput(-speedRps / Constants.kFalconFreeSpeedRPS);
	}

	@Override
	public void execute() {
		if (Math.abs(shooter.getShooterVelocity() - speedRps) <= 10.0d) {
			hasReachedVel = true;
		}
		if (hasReachedVel) {
			shooter.rawIndexer(-Constants.Shooter.kIndexerSpinSpeed);
		}
	}

	@Override
	public void end(boolean wasInterrupted) {
		timer.stop();
		timer.reset();
		hasReachedVel = false;
		shooter.setShooterOutput(0);
		shooter.rawIndexer(0);
	}

	@Override
	public boolean isFinished() {
		return hasReachedVel && timer.get() > 2.0d;
	}
}