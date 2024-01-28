package com.redstorm509.alice2024.commands;

import com.redstorm509.alice2024.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeNote extends Command {
	private final Intake m_Intake;

	public IntakeNote(Intake intake) {
		m_Intake = intake;
		addRequirements(m_Intake);
	}

	@Override
	public void execute() {
		m_Intake.intake(0);
	}
}
