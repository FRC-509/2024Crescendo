package com.redstorm509.alice2024.commands.autonomous;

import com.redstorm509.alice2024.Constants;
import com.redstorm509.alice2024.subsystems.Indexer.IndexerState;
import com.redstorm509.alice2024.subsystems.Indexer;
import com.redstorm509.alice2024.subsystems.Intake;
import com.redstorm509.alice2024.util.drivers.REVBlinkin;
import com.redstorm509.alice2024.util.drivers.REVBlinkin.ColorCode;

import edu.wpi.first.wpilibj2.command.Command;

public class AutonomousIntakeNote extends Command {
	private final Intake intake;
	private final Indexer indexer;
	private final REVBlinkin lights;

	private boolean isFinished = false;

	public AutonomousIntakeNote(Intake intake, Indexer indexer, REVBlinkin lights) {
		this.intake = intake;
		this.indexer = indexer;
		this.lights = lights;

		addRequirements(intake, indexer);
	}

	@Override
	public void initialize() {
		isFinished = false;
	}

	@Override
	public void execute() {
		if (indexer.indexingNoteState == IndexerState.HasNote) {
			isFinished = true;

			lights.setColor(ColorCode.HasNote);
		} else if (indexer.indexingNoteState == IndexerState.Noteless) {
			indexer.rawIndexer(-Constants.Indexer.kSpinSpeed);
			intake.intake(true);

			lights.setColor(ColorCode.NoteInsideRobot);
		} else if (indexer.indexingNoteState == IndexerState.NoteTooShooter) {
			indexer.rawIndexer(Constants.Indexer.kReducedSpinSpeed); // increase if needed
			intake.intake(true);

			lights.setColor(ColorCode.NoteInsideRobot);
		} else if (indexer.indexingNoteState == IndexerState.NoteTooShooterExtreme) {
			indexer.rawIndexer(Constants.Indexer.kSpinSpeed);
			intake.intake(true);

			lights.setColor(ColorCode.NoteInsideRobot);
		} else if (indexer.indexingNoteState == IndexerState.NoteTooIntake) {
			indexer.rawIndexer(-Constants.Indexer.kReducedSpinSpeed); // increase if needed
			intake.stop();

			lights.setColor(ColorCode.NoteInsideRobot);
		} else if (indexer.indexingNoteState == IndexerState.NoteTooIntakeExtreme) {
			indexer.rawIndexer(-Constants.Indexer.kSpinSpeed);
			intake.intake(true);

			lights.setColor(ColorCode.NoteInsideRobot);
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
		if (indexer.indexingNoteState == IndexerState.HasNote) {
			lights.setColor(ColorCode.HasNote);
		} else {
			lights.setDefault();
		}
	}
}
