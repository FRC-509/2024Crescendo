package com.redstorm509.alice2024.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {

	public enum IndexerState {
		HasNote,
		Noteless,
		NoteTooShooter,
		NoteTooShooterExtreme,
		NoteTooIntake,
		NoteTooIntakeExtreme,
	}

	private CANSparkMax indexer = new CANSparkMax(12, MotorType.kBrushed);
	private DigitalInput shooterBB = new DigitalInput(1); // CHANGE TO REAL PORTS
	private DigitalInput indexerBB = new DigitalInput(2);
	private DigitalInput imStageBB = new DigitalInput(3);

	public Indexer() {
		indexer.setSmartCurrentLimit(15);
		indexer.setIdleMode(IdleMode.kCoast);
		indexer.burnFlash();
	}

	public void rawIndexer(double speed) {
		indexer.set(speed);
	}

	public boolean hasNote() {
		return indexingNoteState() == IndexerState.HasNote;
	}

	public IndexerState indexingNoteState() {
		if (indexerBB.get() && shooterBB.get() && !imStageBB.get()) {
			// Note is where we want it to be
			return IndexerState.HasNote;
		} else if (indexerBB.get() && !shooterBB.get() && !imStageBB.get()) {
			// Note is too far out shooter side
			return IndexerState.NoteTooShooter;
		} else if (indexerBB.get() && !shooterBB.get() && imStageBB.get()) {
			// Note is too far out intermediate stage side
			return IndexerState.NoteTooIntake;
		} else if (indexerBB.get() && !shooterBB.get() && !imStageBB.get()) {
			// Note is inside of indexer, but not far enough
			return IndexerState.NoteTooIntake;
		} else if (!indexerBB.get() && shooterBB.get() && !imStageBB.get()) {
			// Note is way too sticking out shooter
			return IndexerState.NoteTooShooterExtreme;
		} else if (!indexerBB.get() && !shooterBB.get() && imStageBB.get()) {
			// Note is way too sticking out shooter
			return IndexerState.NoteTooIntakeExtreme;
		} else {
			// Does not have a note, or is invalid state
			return IndexerState.Noteless;
		}
	}
}
