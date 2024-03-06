package com.redstorm509.alice2024.commands;

import com.redstorm509.alice2024.Constants;
import com.redstorm509.alice2024.subsystems.Indexer.IndexerState;
import com.redstorm509.alice2024.subsystems.Indexer;
import com.redstorm509.alice2024.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeNote extends Command {
	private final Intake intake;
	private final Indexer indexer;
	private boolean isFinished = false;

	public IntakeNote(Intake intake, Indexer indexer) {
		this.intake = intake;
		this.indexer = indexer;

		addRequirements(intake, indexer);
	}

	@Override
	public void initialize() {
		isFinished = false;
	}

	@Override
	public void execute() {
		IndexerState indexerState = indexer.indexingNoteState();

		if (indexerState == IndexerState.HasNote) {
			isFinished = true;
		} else if (indexerState == IndexerState.Noteless) {
			indexer.rawIndexer(-Constants.Shooter.kIndexerSpinSpeed);
			intake.intake(true);
		} else if (indexerState == IndexerState.NoteTooShooter) {
			indexer.rawIndexer(Constants.Shooter.kIndexerSpinSpeed * 0.5); // increase if needed
			intake.stop();
		} else if (indexerState == IndexerState.NoteTooShooterExtreme) {
			indexer.rawIndexer(Constants.Shooter.kIndexerSpinSpeed);
			intake.stop();
		} else if (indexerState == IndexerState.NoteTooIntake) {
			indexer.rawIndexer(-Constants.Shooter.kIndexerSpinSpeed * 0.5); // increase if needed
			intake.stop();
		} else if (indexerState == IndexerState.NoteTooIntakeExtreme) {
			indexer.rawIndexer(-Constants.Shooter.kIndexerSpinSpeed);
			intake.intake(true);
		}
	}

	@Override
	public boolean isFinished() {
		return isFinished;
	}

	@Override
	public void end(boolean wasInterrupted) {
		indexer.rawIndexer(0.0);
		intake.stop();
	}
}
