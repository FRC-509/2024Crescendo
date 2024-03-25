package com.redstorm509.alice2024.commands.autonomous;

import com.ctre.phoenix6.controls.VoltageOut;
import com.redstorm509.alice2024.Constants;
import com.redstorm509.alice2024.subsystems.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoShootJank extends Command {
	private Shooter shooter;
	private Indexer indexer;
	private Timer timer = new Timer();
	private boolean firstReached = false;
	private boolean startedindexing = false;

	private boolean isFinished = false;

	public AutoShootJank(Shooter shooter, Indexer indexer) {
		this.shooter = shooter;
		this.indexer = indexer;

		addRequirements(indexer, shooter);
	}

	@Override
	public void initialize() {
		shooter.setShooterVelocity(-Constants.Shooter.kTargetSpeed);
		timer.reset();
		timer.start();
		firstReached = false;
		startedindexing = false;

	}

	@Override
	public void execute() {
		boolean atSpeed = Math.abs(Math.abs(shooter.getShooterVelocity()) - Constants.Shooter.kTargetSpeed) <= 2.0d;
		if (atSpeed && !firstReached) {
			firstReached = true;
			timer.reset();
		}
		if (firstReached && timer.get() > 1.6) {
			indexer.rawIndexer(Constants.Indexer.kShootSpeed);
			startedindexing = true;
		}

		if (timer.get() >= 3.0 && startedindexing == true) {
			isFinished = true;
		}
	}

	@Override
	public void end(boolean wasInterrupted) {
		shooter.shooterLeader.setControl(new VoltageOut(0.0));
		indexer.rawIndexer(0.0);
	}

	@Override
	public boolean isFinished() {
		return isFinished;
	}
}