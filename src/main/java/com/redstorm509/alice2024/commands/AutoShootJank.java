package com.redstorm509.alice2024.commands;

import com.fasterxml.jackson.databind.ser.AnyGetterWriter;
import com.redstorm509.alice2024.Constants;
import com.redstorm509.alice2024.subsystems.Indexer;
import com.redstorm509.alice2024.subsystems.Shooter;
import com.redstorm509.alice2024.subsystems.Indexer.IndexerState;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoShootJank extends Command {
	private Shooter shooter;
	private Indexer indexer;
	private Timer timer = new Timer();
	private boolean firstReached = false;

	private boolean isFinished = false;

	public AutoShootJank(Shooter shooter, Indexer indexer) {
		this.shooter = shooter;
		this.indexer = indexer;

		addRequirements(indexer, shooter);
	}

	@Override
	public void initialize() {
		shooter.setShooterVelocity(-Constants.Shooter.kTargetSpeed);
		timer.start();
		firstReached = false;
	}

	@Override
	public void execute() {
		boolean atSpeed = Math.abs(Math.abs(shooter.getShooterVelocity()) - Constants.Shooter.kTargetSpeed) <= 2.0d;
		if (atSpeed && !firstReached) {
			firstReached = true;
			timer.reset();
		} else if (firstReached && timer.get() > 1) {
			indexer.rawIndexer(-1.0);
			if (timer.get() >= 3.0) {
				isFinished = true;
			}
		}
	}

	@Override
	public void end(boolean wasInterrupted) {
		shooter.setShooterVelocity(0.0);
		indexer.rawIndexer(0.0);
		indexer.setNoteless();
	}

	@Override
	public boolean isFinished() {
		return isFinished;
	}
}