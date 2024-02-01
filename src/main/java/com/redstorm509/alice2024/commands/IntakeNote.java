package com.redstorm509.alice2024.commands;

import com.redstorm509.alice2024.Constants;
import com.redstorm509.alice2024.subsystems.Intake;
import com.redstorm509.alice2024.subsystems.PreCompressor;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeNote extends Command {
	private final Intake m_Intake;
	private final PreCompressor m_PreCompressor;

	public IntakeNote(Intake intake, PreCompressor preCompressor) {
		m_Intake = intake;
		m_PreCompressor = preCompressor;
		addRequirements(m_Intake, m_PreCompressor);
	}

	@Override
	public void execute() {
		m_Intake.intake(Constants.Intake.intakeSpinSpeed);
		// m_Intake.runOnce();
	}
}
