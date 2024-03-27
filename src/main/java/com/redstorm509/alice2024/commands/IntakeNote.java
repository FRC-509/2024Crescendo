package com.redstorm509.alice2024.commands;

import com.redstorm509.alice2024.Constants;
import com.redstorm509.alice2024.subsystems.Indexer.IndexerState;
import com.redstorm509.alice2024.util.drivers.REVBlinkin;
import com.redstorm509.alice2024.util.drivers.REVBlinkin.ColorCode;
import com.redstorm509.alice2024.subsystems.Indexer;
import com.redstorm509.alice2024.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeNote extends Command {
	private final Intake intake;
	private final Indexer indexer;
	private final REVBlinkin lights;

	private boolean isFinished = false;
	private double fastSpeed = Constants.Indexer.kSpinSpeed;
	private double slowSpeed = Constants.Indexer.kReducedSpinSpeed;

	public IntakeNote(Intake intake, Indexer indexer, REVBlinkin underglow) {
		this.intake = intake;
		this.indexer = indexer;
		this.lights = underglow;

		addRequirements(intake, indexer);
	}

	@Override
	public void initialize() {
		isFinished = false;
		lights.setDefault();
	}

	@Override
	public void execute() {
		if (indexer.indexingNoteState == IndexerState.HasNote) {
			isFinished = true;

			lights.setColor(ColorCode.HasNote);
		} else if (indexer.indexingNoteState == IndexerState.Noteless) {
			indexer.rawIndexer(-fastSpeed);
			intake.intake(true);

			lights.setColor(ColorCode.NoteInsideRobot);
		} else if (indexer.indexingNoteState == IndexerState.NoteTooShooter) {
			indexer.rawIndexer(slowSpeed); // increase if needed
			intake.stop();

			lights.setColor(ColorCode.NoteInsideRobot);
		} else if (indexer.indexingNoteState == IndexerState.NoteTooShooterExtreme) {
			indexer.rawIndexer(fastSpeed);
			intake.stop();

			lights.setColor(ColorCode.NoteInsideRobot);
		} else if (indexer.indexingNoteState == IndexerState.NoteTooIntake) {
			indexer.rawIndexer(-slowSpeed); // increase if needed
			intake.stop();

			lights.setColor(ColorCode.NoteInsideRobot);
		} else if (indexer.indexingNoteState == IndexerState.NoteTooIntakeExtreme) {
			indexer.rawIndexer(-fastSpeed);
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
