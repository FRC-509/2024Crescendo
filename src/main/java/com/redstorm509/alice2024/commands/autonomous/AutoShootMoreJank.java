package com.redstorm509.alice2024.commands.autonomous;

import com.redstorm509.alice2024.Constants;
import com.redstorm509.alice2024.subsystems.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoShootMoreJank extends Command {
	private Shooter shooter;
	private Indexer indexer;
	private Timer timer = new Timer();
	private boolean startedindexing = false;

	private boolean isFinished = false;

	public AutoShootMoreJank(Shooter shooter, Indexer indexer) {
		this.shooter = shooter;
		this.indexer = indexer;

		addRequirements(indexer, shooter);
	}

	@Override
	public void initialize() {
		timer.reset();
		timer.start();
		startedindexing = false;
	}

	@Override
	public void execute() {
		boolean atSpeed = Math.abs(Math.abs(shooter.getShooterVelocity()) - Constants.Shooter.kTargetSpeed) <= 4.0d;
		if (atSpeed && !startedindexing) {
			indexer.rawIndexer(Constants.Indexer.kShootSpeed);
			startedindexing = true;
		}

		if (timer.get() >= 0.75 && startedindexing == true) {
			isFinished = true;
		}
	}

	@Override
	public void end(boolean wasInterrupted) {
		indexer.rawIndexer(0.0);
	}

	@Override
	public boolean isFinished() {
		return isFinished;
	}
}