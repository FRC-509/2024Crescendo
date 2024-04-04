package com.redstorm509.alice2024.commands.autonomous;

import com.redstorm509.alice2024.Constants;
import com.redstorm509.alice2024.subsystems.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class AutonomousRemoveNoteShooter extends Command {
	private Shooter shooter;
	private Indexer indexer;

	private Timer timer = new Timer();
	private boolean startedindexing = false;
	private double reducedShootSpeed = Constants.Shooter.kTargetSpeed * 0.07;

	private boolean isFinished = false;

	public AutonomousRemoveNoteShooter(Shooter shooter, Indexer indexer) {
		this.shooter = shooter;
		this.indexer = indexer;

		addRequirements(indexer, shooter);
	}

	@Override
	public void initialize() {
		timer.reset();
		timer.start();
		startedindexing = false;
		shooter.setShooterVelocity(reducedShootSpeed);
	}

	@Override
	public void execute() {
		boolean atSpeed = Math.abs(Math.abs(shooter.getShooterVelocity()) - reducedShootSpeed) <= 4.0d;
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
		shooter.setShooterVelocity(-Constants.Shooter.kTargetSpeed);
	}

	@Override
	public boolean isFinished() {
		return isFinished;
	}
}
