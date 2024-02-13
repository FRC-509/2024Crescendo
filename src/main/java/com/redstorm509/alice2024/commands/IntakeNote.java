package com.redstorm509.alice2024.commands;

import com.redstorm509.alice2024.subsystems.Intake;
import com.redstorm509.alice2024.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeNote extends Command {
	private final Intake m_Intake;
	private final Shooter shooter;

	public IntakeNote(Intake intake, Shooter shooter) {
		m_Intake = intake;
		this.shooter = shooter;

		addRequirements(m_Intake);
	}

	@Override
	public void execute() {
		if (shooter.hasIntaken()) {
			end(true);
		}

		m_Intake.intake(true);
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
