package com.redstorm509.alice2024.commands;

import java.io.Console;

import com.redstorm509.alice2024.Constants;
import com.redstorm509.alice2024.subsystems.Intake;
import com.redstorm509.alice2024.util.TimeStamp;
import com.redstorm509.alice2024.util.devices.VL53L4CD;
import com.redstorm509.alice2024.util.devices.VL53L4CD.Measurement;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeNote extends Command {
	private final Intake m_Intake;
	private final VL53L4CD initialToF;
	private final VL53L4CD secondaryToF;

	private TimeStamp timeStamp;

	public IntakeNote(Intake intake, VL53L4CD initialToF, VL53L4CD secondaryToF) {
		m_Intake = intake;
		this.initialToF = initialToF;
		this.secondaryToF = secondaryToF;

		addRequirements(m_Intake);
	}

	@Override
	public void execute() {
		m_Intake.intake(true);

		// not finished
		if (initialToF.measure().distanceMillimeters == Constants.Devices.kToFNoteDetectionThreshold) {
			timeStamp = new TimeStamp();
		}
		// change time value to relevent time, or dont idk
		if (secondaryToF.measure().distanceMillimeters == Constants.Devices.kToFNoteDetectionThreshold
				&& timeStamp.deltaTime() < 1.0) {
			end(false);
		}
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean wasInterrupted) {
		m_Intake.stop();
	}
}
