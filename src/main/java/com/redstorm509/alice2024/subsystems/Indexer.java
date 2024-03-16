package com.redstorm509.alice2024.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import org.opencv.features2d.FastFeatureDetector;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {

	public enum IndexerState {
		HasNote,
		Noteless,
		NoteTooShooter,
		NoteTooShooterExtreme,
		NoteTooIntake,
		NoteTooIntakeExtreme;

		public String toString() {
			switch (this) {
				case HasNote:
					return "Has Note";
				case NoteTooIntake:
					return "Note is too close to the intake!";
				case NoteTooIntakeExtreme:
					return "Note is WAY too close to the intake!";
				case NoteTooShooter:
					return "Note is too close to the shooter!";
				case NoteTooShooterExtreme:
					return "Note is WAY too close to the shooter!";
				case Noteless:
					return "Noteless";
				default:
					return "Invalid";
			}
		}
	}

	public IndexerState indexingNoteState;
	private IndexerState prevIndexerState;
	public boolean ignoreBBLogic = true;

	public boolean indexerInvalidState;
	private Timer currentStateTimer = new Timer();

	private CANSparkMax indexer = new CANSparkMax(12, MotorType.kBrushed);
	private DigitalInput shooterBB = new DigitalInput(4); // CHANGE TO REAL PORTS
	private DigitalInput indexerBB = new DigitalInput(2);
	private DigitalInput imStageBB = new DigitalInput(0);

	public Indexer() {
		indexer.setSmartCurrentLimit(30);
		indexer.setIdleMode(IdleMode.kCoast);
		indexer.burnFlash();
		indexingNoteState = IndexerState.Noteless;
		indexerInvalidState = false;
		ignoreBBLogic = true;
		currentStateTimer.start();
	}

	public void rawIndexer(double speed) {
		indexer.set(speed);
	}

	public boolean hasNote() {
		return indexingNoteState == IndexerState.HasNote;
	}

	public void setNoteless() {
		indexingNoteState = IndexerState.Noteless;
	}

	public void setHasNote() {
		indexingNoteState = IndexerState.HasNote;
	}

	@Override
	public void periodic() {
		if (!ignoreBBLogic) {
			if (!indexerBB.get() && !shooterBB.get() && imStageBB.get()) {
				// Note is where we want it to be
				prevIndexerState = indexingNoteState;
				indexingNoteState = IndexerState.HasNote;
				indexerInvalidState = false;
			} else if (!indexerBB.get() && shooterBB.get() && imStageBB.get()) {
				// Note is too far out shooter side
				prevIndexerState = indexingNoteState;
				indexingNoteState = IndexerState.NoteTooShooter;
				indexerInvalidState = false;
			} else if (!indexerBB.get() && shooterBB.get() && !imStageBB.get()) {
				// Note is too far out intermediate stage side
				prevIndexerState = indexingNoteState;
				indexingNoteState = IndexerState.NoteTooIntake;
				indexerInvalidState = false;
			} else if (indexerBB.get() && !shooterBB.get() && imStageBB.get()) {
				// Note is way too sticking out shooter
				prevIndexerState = indexingNoteState;
				indexingNoteState = IndexerState.NoteTooShooterExtreme;
				indexerInvalidState = false;
			} else if (indexerBB.get() && shooterBB.get() && !imStageBB.get()) {
				// Note is way too sticking out shooter
				prevIndexerState = indexingNoteState;
				indexingNoteState = IndexerState.NoteTooIntakeExtreme;
				indexerInvalidState = false;
			}
		}

		if (indexingNoteState == prevIndexerState && indexingNoteState != IndexerState.NoteTooShooter) {
			currentStateTimer.reset();
		}
		if (indexingNoteState == IndexerState.NoteTooShooter && currentStateTimer.get() >= 0.75) {
			setNoteless();
		}
		SmartDashboard.putBoolean("Note Picked Up", indexingNoteState != IndexerState.Noteless);
		SmartDashboard.putBoolean("Has Note", indexingNoteState == IndexerState.HasNote);
		// SmartDashboard.putBoolean("Is IM Stage BB Tripped?", !imStageBB.get());
		// SmartDashboard.putBoolean("Is Indexer BB Tripped?", !indexerBB.get());
		// SmartDashboard.putBoolean("Is Shooter BB Tripped?", !shooterBB.get());

	}
}