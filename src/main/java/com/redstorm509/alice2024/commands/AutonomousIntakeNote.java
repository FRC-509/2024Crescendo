package com.redstorm509.alice2024.commands;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.redstorm509.alice2024.Constants;
import com.redstorm509.alice2024.subsystems.ArmRS;
import com.redstorm509.alice2024.subsystems.Indexer.IndexerState;
import com.redstorm509.alice2024.subsystems.vision.Limelight;
import com.redstorm509.alice2024.subsystems.Indexer;
import com.redstorm509.alice2024.subsystems.Intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AutonomousIntakeNote extends Command {
	private final Intake intake;
	private final Indexer indexer;
	private boolean isFinished = false;
	private double fastSpeed = Constants.Indexer.kSpinSpeed;
	private double slowSpeed = Constants.Indexer.kReducedSpinSpeed;

	public AutonomousIntakeNote(Intake intake, Indexer indexer) {
		this.intake = intake;
		this.indexer = indexer;

		addRequirements(intake, indexer);

		SmartDashboard.putNumber("Fast Indexer Speed", Constants.Indexer.kSpinSpeed);
		SmartDashboard.putNumber("Slow Indexer Speed", Constants.Indexer.kReducedSpinSpeed);
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
	}

	@Override
	public void execute() {
		fastSpeed = SmartDashboard.getNumber("Fast Indexer Speed", fastSpeed);
		slowSpeed = SmartDashboard.getNumber("Slow Indexer Speed", slowSpeed);

		if (indexer.indexingNoteState == IndexerState.HasNote) {
			isFinished = true;
		} else if (indexer.indexingNoteState == IndexerState.Noteless) {
			indexer.rawIndexer(-Constants.Indexer.kSpinSpeed); 
			intake.intake(true);
		} else if (indexer.indexingNoteState == IndexerState.NoteTooShooter) {
			indexer.rawIndexer(Constants.Indexer.kReducedSpinSpeed); // increase if needed
			intake.intake(true);
		} else if (indexer.indexingNoteState == IndexerState.NoteTooShooterExtreme) {
			indexer.rawIndexer(Constants.Indexer.kSpinSpeed);
			intake.intake(true);
		} else if (indexer.indexingNoteState == IndexerState.NoteTooIntake) {
			indexer.rawIndexer(-Constants.Indexer.kReducedSpinSpeed); // increase if needed
			intake.intake(true);
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
		}
	}
}
