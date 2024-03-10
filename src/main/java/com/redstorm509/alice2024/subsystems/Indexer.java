package com.redstorm509.alice2024.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
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

	private CANSparkMax indexer = new CANSparkMax(12, MotorType.kBrushed);
	private DigitalInput shooterBB = new DigitalInput(4); // CHANGE TO REAL PORTS
	private DigitalInput indexerBB = new DigitalInput(2);
	private DigitalInput imStageBB = new DigitalInput(0);

	public Indexer() {
		indexer.setSmartCurrentLimit(15);
		indexer.setIdleMode(IdleMode.kCoast);
		indexer.burnFlash();
		indexingNoteState = IndexerState.Noteless;
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

	@Override
	public void periodic() {
		if (!indexerBB.get() && !shooterBB.get() && imStageBB.get()) {
			// Note is where we want it to be
			indexingNoteState = IndexerState.HasNote;
		} else if (!indexerBB.get() && shooterBB.get() && imStageBB.get()) {
			// Note is too far out shooter side
			indexingNoteState = IndexerState.NoteTooShooter;
		} else if (!indexerBB.get() && shooterBB.get() && !imStageBB.get()) {
			// Note is too far out intermediate stage side
			indexingNoteState = IndexerState.NoteTooIntake;
		} else if (indexerBB.get() && !shooterBB.get() && imStageBB.get()) {
			// Note is way too sticking out shooter
			indexingNoteState = IndexerState.NoteTooShooterExtreme;
		} else if (indexerBB.get() && shooterBB.get() && !imStageBB.get()) {
			// Note is way too sticking out shooter
			indexingNoteState = IndexerState.NoteTooIntakeExtreme;
		}

		SmartDashboard.putString("Indexer State", indexingNoteState.toString());
	}
}