package com.redstorm509.alice2024.commands.autonomous;

import com.redstorm509.alice2024.Constants;
import com.redstorm509.alice2024.subsystems.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class AutonomousShootEvenMoreJankButItsOk extends Command {
	private Shooter shooter;
	private Indexer indexer;

	private Timer timer = new Timer();
	private boolean startedindexing = false;
	private double targetSpeed;

	private boolean isFinished = false;

	public AutonomousShootEvenMoreJankButItsOk(double targetSpeed, Shooter shooter, Indexer indexer) {
		this.shooter = shooter;
		this.indexer = indexer;
		this.targetSpeed = targetSpeed;

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
		boolean atSpeed = Math.abs(shooter.getShooterVelocity()) - 7.0 >= Math.abs(targetSpeed);
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
	}

	@Override
	public boolean isFinished() {
		return isFinished;
	}
}
