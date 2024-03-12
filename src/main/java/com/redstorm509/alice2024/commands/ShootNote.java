package com.redstorm509.alice2024.commands;

import com.redstorm509.alice2024.Constants;
import com.redstorm509.alice2024.subsystems.Indexer;
import com.redstorm509.alice2024.subsystems.Shooter;
import com.redstorm509.alice2024.subsystems.Indexer.IndexerState;

import edu.wpi.first.wpilibj2.command.Command;

public class ShootNote extends Command {
	private Shooter shooter;
	private Indexer indexer;

	private boolean isFinished = false;

	public ShootNote(Shooter shooter, Indexer indexer) {
		this.shooter = shooter;
		this.indexer = indexer;

		addRequirements(indexer, shooter);
	}

	@Override
	public void initialize() {
		System.out.println("Started shooting!");
		isFinished = !indexer.hasNote();
	}

	@Override
	public void execute() {
		System.out.println("shooting!");
		shooter.setShooterVelocity(-Constants.Shooter.kTargetSpeed);

		if (Math.abs(shooter.getShooterVelocity() + Constants.Shooter.kTargetSpeed) <= 3.0d) {
			indexer.rawIndexer(-Constants.Indexer.kSpinSpeed);
		}

		if (indexer.indexingNoteState == IndexerState.Noteless) {
			isFinished = true;
		}
	}

	@Override
	public void end(boolean wasInterrupted) {
		System.out.println("Done shooting!");
		shooter.setShooterVelocity(0.0);
		indexer.rawIndexer(0.0);
		indexer.setNoteless();
	}

	@Override
	public boolean isFinished() {
		return isFinished;
	}
}