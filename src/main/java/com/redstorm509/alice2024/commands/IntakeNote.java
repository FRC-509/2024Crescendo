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

public class IntakeNote extends Command {
	private final Intake intake;
	private final Indexer indexer;
	private final ArmRS arm;
	private boolean isFinished = false;
	private double fastSpeed = Constants.Indexer.kSpinSpeed;
	private double slowSpeed = Constants.Indexer.kReducedSpinSpeed;

	public IntakeNote(Intake intake, Indexer indexer, ArmRS arm) {
		this.intake = intake;
		this.indexer = indexer;
		this.arm = arm;

		addRequirements(intake, indexer, arm);

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
			indexer.rawIndexer(-fastSpeed);
			intake.intake(true);
		} else if (indexer.indexingNoteState == IndexerState.NoteTooShooter) {
			indexer.rawIndexer(slowSpeed); // increase if needed
			intake.stop();
		} else if (indexer.indexingNoteState == IndexerState.NoteTooShooterExtreme) {
			indexer.rawIndexer(fastSpeed);
			intake.stop();
		} else if (indexer.indexingNoteState == IndexerState.NoteTooIntake) {
			indexer.rawIndexer(-slowSpeed); // increase if needed
			intake.stop();
		} else if (indexer.indexingNoteState == IndexerState.NoteTooIntakeExtreme) {
			indexer.rawIndexer(-fastSpeed);
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
