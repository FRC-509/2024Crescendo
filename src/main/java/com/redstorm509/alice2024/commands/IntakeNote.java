package com.redstorm509.alice2024.commands;

import com.redstorm509.alice2024.Constants;
import com.redstorm509.alice2024.subsystems.ArmIS;
import com.redstorm509.alice2024.subsystems.Indexer.IndexerState;
import com.redstorm509.alice2024.subsystems.Indexer;
import com.redstorm509.alice2024.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeNote extends Command {
	private final Intake intake;
	private final Indexer indexer;
	private final ArmIS arm;
	private boolean isFinished = false;

	public IntakeNote(Intake intake, Indexer indexer, ArmIS arm) {
		this.intake = intake;
		this.indexer = indexer;
		this.arm = arm;

		addRequirements(intake, indexer, arm);
	}

	@Override
	public void initialize() {
		/*-
		if (arm.isLimitSwitchTripped()) {
			isFinished = true;
		} else {
			isFinished = false;
			indexer.ignoreBBLogic = false;
		}
		 */
		isFinished = false;
		indexer.ignoreBBLogic = false;
	}

	@Override
	public void execute() {
		if (indexer.indexingNoteState == IndexerState.HasNote) {
			isFinished = true;
		} else if (indexer.indexingNoteState == IndexerState.Noteless) {
			indexer.rawIndexer(-Constants.Indexer.kSpinSpeed);
			intake.intake(true);
		} else if (indexer.indexingNoteState == IndexerState.NoteTooShooter) {
			indexer.rawIndexer(Constants.Indexer.kReducedSpinSpeed); // increase if needed
			intake.stop();
		} else if (indexer.indexingNoteState == IndexerState.NoteTooShooterExtreme) {
			indexer.rawIndexer(Constants.Indexer.kSpinSpeed);
			intake.stop();
		} else if (indexer.indexingNoteState == IndexerState.NoteTooIntake) {
			indexer.rawIndexer(-Constants.Indexer.kReducedSpinSpeed); // increase if needed
			intake.stop();
		} else if (indexer.indexingNoteState == IndexerState.NoteTooIntakeExtreme) {
			indexer.rawIndexer(-Constants.Indexer.kSpinSpeed);
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
		if (indexer.indexingNoteState == IndexerState.HasNote) {
			indexer.ignoreBBLogic = true;
		}
	}
}
