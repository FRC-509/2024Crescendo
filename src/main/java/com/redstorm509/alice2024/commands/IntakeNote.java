package com.redstorm509.alice2024.commands;

import com.redstorm509.alice2024.Constants;
import com.redstorm509.alice2024.subsystems.Intake;
import com.redstorm509.alice2024.subsystems.Shooter;
import com.redstorm509.alice2024.subsystems.Shooter.IndexerState;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeNote extends Command {
	private final Intake intake;
	private final Shooter shooter;
	private boolean isFinished = false;

	public IntakeNote(Intake intake, Shooter shooter) {
		this.intake = intake;
		this.shooter = shooter;

		addRequirements(intake, shooter);
	}

	@Override
	public void initialize() {
		isFinished = false;
	}

	@Override
	public void execute() {
		IndexerState indexerState = shooter.indexingNoteState();

		// negative indexer is towards shooter
		if (indexerState == IndexerState.HasNote) {
			isFinished = true;
		} else if (indexerState == IndexerState.Noteless) {
			shooter.rawIndexer(-Constants.Shooter.kIndexerSpinSpeed);
			intake.intake(true);
		} else if (indexerState == IndexerState.NoteTooShooter) {
			shooter.rawIndexer(Constants.Shooter.kIndexerSpinSpeed * 0.5); // increase if needed
			intake.stop();
		} else if (indexerState == IndexerState.NoteTooShooterExtreme) {
			shooter.rawIndexer(Constants.Shooter.kIndexerSpinSpeed);
			intake.stop();
		} else if (indexerState == IndexerState.NoteTooIntake) {
			shooter.rawIndexer(-Constants.Shooter.kIndexerSpinSpeed * 0.5); // increase if needed
			intake.stop();
		} else if (indexerState == IndexerState.NoteTooIntakeExtreme) {
			shooter.rawIndexer(-Constants.Shooter.kIndexerSpinSpeed);
			intake.intake(true);
		}
	}

	@Override
	public boolean isFinished() {
		return isFinished;
	}

	@Override
	public void end(boolean wasInterrupted) {
		shooter.rawIndexer(0.0);
		intake.stop();
	}
}
