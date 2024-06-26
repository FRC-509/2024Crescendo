package com.redstorm509.alice2024.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.function.BooleanSupplier;

import com.redstorm509.alice2024.util.telemetry.ThinNT;
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
	private Timer currentStateTimer = new Timer();
	private boolean isInvalidState;
	private BooleanSupplier armIsDown;

	private CANSparkMax indexer = new CANSparkMax(12, MotorType.kBrushed);
	private DigitalInput shooterBB = new DigitalInput(4);
	private DigitalInput indexerBB = new DigitalInput(2);
	private DigitalInput imStageBB = new DigitalInput(0);

	public Indexer(BooleanSupplier armIsDown) {
		indexer.setSmartCurrentLimit(30);
		indexer.setIdleMode(IdleMode.kCoast);
		indexer.burnFlash();
		indexingNoteState = IndexerState.Noteless;
		prevIndexerState = IndexerState.Noteless;
		isInvalidState = false;
		this.armIsDown = armIsDown;

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

	public boolean isInvalidState() {
		return isInvalidState;
	}

	public void pollState() {
		if (armIsDown.getAsBoolean()) {
			if (!indexerBB.get() && !shooterBB.get() && !imStageBB.get()) {
				// Note is where we want it to be
				prevIndexerState = indexingNoteState;
				indexingNoteState = IndexerState.HasNote;
				isInvalidState = false;
			} else if (!indexerBB.get() && !shooterBB.get() && imStageBB.get()) {
				// Note is too far out shooter side
				prevIndexerState = indexingNoteState;
				indexingNoteState = IndexerState.NoteTooShooter;
				isInvalidState = false;
			} else if (!indexerBB.get() && shooterBB.get() && !imStageBB.get()) {
				// Note is too far out intermediate stage side
				prevIndexerState = indexingNoteState;
				indexingNoteState = IndexerState.NoteTooIntake;
				isInvalidState = false;
			} else if (indexerBB.get() && !shooterBB.get() && imStageBB.get()) {
				// Note is way too sticking out shooter
				prevIndexerState = indexingNoteState;
				indexingNoteState = IndexerState.NoteTooShooterExtreme;
				isInvalidState = false;
			} else if (indexerBB.get() && shooterBB.get() && !imStageBB.get()) {
				// Note is way too close to the intake
				prevIndexerState = indexingNoteState;
				indexingNoteState = IndexerState.NoteTooIntakeExtreme;
				isInvalidState = false;
			} else {
				isInvalidState = true;
			}
		} else {
			prevIndexerState = indexingNoteState;
			indexingNoteState = getIndexerOnlyState();
			isInvalidState = false;
		}

		if (indexingNoteState != prevIndexerState) {
			currentStateTimer.reset();
		}

		if (indexingNoteState != IndexerState.Noteless && indexingNoteState != IndexerState.HasNote
				&& currentStateTimer.hasElapsed(0.50)) {
			setNoteless();
		}
	}

	public IndexerState getIndexerOnlyState() {
		if (!shooterBB.get() && !indexerBB.get()) {
			// Note is where we want it to be
			return IndexerState.HasNote;
		} else if (shooterBB.get() && indexerBB.get()) {
			// this creates a small area that it thinks its noteless, but it is so low in
			// the indexer that it will never occur
			return IndexerState.Noteless;
		}
		return IndexerState.NoteTooIntake;
	}

	public boolean noSensorsTripped() {
		return indexerBB.get() && shooterBB.get() && imStageBB.get();
	}

	public boolean isNoteInsideIndexer() {
		return indexingNoteState != IndexerState.Noteless && indexingNoteState != IndexerState.NoteTooIntakeExtreme
				&& indexingNoteState != IndexerState.NoteTooIntake;
	}

	public boolean isNoteInsideRobot() {
		return indexingNoteState != IndexerState.Noteless;
	}

	@Override
	public void periodic() {
		pollState();
		ThinNT.putNumber("IndexerDutyCycle", indexer.get());

		SmartDashboard.putBoolean("Note Picked Up", indexingNoteState != IndexerState.Noteless);
		SmartDashboard.putBoolean("Has Note", indexingNoteState == IndexerState.HasNote);
		SmartDashboard.putString("IndexingState", indexingNoteState.toString());

		SmartDashboard.putBoolean("ShooterBB", shooterBB.get());
		SmartDashboard.putBoolean("IndexerBB", indexerBB.get());
		SmartDashboard.putBoolean("IMStageBB", imStageBB.get());
	}
}