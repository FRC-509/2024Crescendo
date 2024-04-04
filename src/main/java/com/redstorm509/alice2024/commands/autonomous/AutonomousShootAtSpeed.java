package com.redstorm509.alice2024.commands.autonomous;

import com.redstorm509.alice2024.Constants;
import com.redstorm509.alice2024.subsystems.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class AutonomousShootAtSpeed extends Command {
	private Shooter shooter;
	private Indexer indexer;

	private Timer timer = new Timer();
	private boolean startedindexing = false;
	private double targetVelocity;
	private double previousVelocity;

	private boolean isFinished = false;

	public AutonomousShootAtSpeed(double targetVelocity, Shooter shooter, Indexer indexer) {
		this.shooter = shooter;
		this.indexer = indexer;
		this.targetVelocity = targetVelocity;

		addRequirements(indexer, shooter);
	}

	@Override
	public void initialize() {
		timer.reset();
		timer.start();
		previousVelocity = shooter.getGoalVelocity();
		shooter.setShooterVelocity(targetVelocity);
		startedindexing = false;
	}

	@Override
	public void execute() {
		boolean atSpeed = Math.abs(Math.abs(shooter.getShooterVelocity()) - targetVelocity) <= 4.0d;
		if (atSpeed && !startedindexing) {
			timer.reset();
			indexer.rawIndexer(Constants.Indexer.kShootSpeed);
			startedindexing = true;
		}

		if (timer.get() >= 0.50 && startedindexing == true) {
			isFinished = true;
		}
	}

	@Override
	public void end(boolean wasInterrupted) {
		indexer.rawIndexer(0.0);
		indexer.setNoteless();
		shooter.setShooterVelocity(previousVelocity);
	}

	@Override
	public boolean isFinished() {
		return isFinished;
	}
}
