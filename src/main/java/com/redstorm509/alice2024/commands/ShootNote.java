package com.redstorm509.alice2024.commands;

import java.util.function.BooleanSupplier;

import com.redstorm509.alice2024.Constants;
import com.redstorm509.alice2024.subsystems.Indexer;
import com.redstorm509.alice2024.subsystems.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootNote extends Command {
	private Shooter shooter;
	private Indexer indexer;
	private double speedRps;
	private boolean hasReachedVel = false;
	private boolean runIndexer = false;
	private BooleanSupplier intakeBooleanSupp;
	private Timer timer;

	public ShootNote(Shooter shooter, Indexer indexer, double speedRotationsPerSecond, boolean runIndexer,
			BooleanSupplier intakeIndexerSupp) {
		this.shooter = shooter;
		this.indexer = indexer;
		this.speedRps = speedRotationsPerSecond;
		this.timer = new Timer();
		this.runIndexer = runIndexer;
		this.intakeBooleanSupp = intakeIndexerSupp;
		addRequirements(shooter);
	}

	@Override
	public void initialize() {
		timer.reset();
		timer.start();
		hasReachedVel = false;
		indexer.rawIndexer(0);
		shooter.setShooterOutput(-speedRps / Constants.kFalconFreeSpeedRPS);
	}

	@Override
	public void execute() {
		if (Math.abs(shooter.getShooterVelocity() + speedRps) <= 10.0d) {
			hasReachedVel = true;
		}
		if (intakeBooleanSupp.getAsBoolean()) {
			indexer.rawIndexer(-Constants.Shooter.kIndexerSpinSpeed);
		}
		if (hasReachedVel && runIndexer) {
			indexer.rawIndexer(-Constants.Shooter.kIndexerSpinSpeed);
		}
	}

	@Override
	public void end(boolean wasInterrupted) {
		timer.stop();
		timer.reset();
		hasReachedVel = false;
		shooter.setShooterOutput(0);
		indexer.rawIndexer(0);
	}

	@Override
	public boolean isFinished() {
		return hasReachedVel && timer.get() > 2.0d;
	}
}