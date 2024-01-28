package com.redstorm509.alice2024.commands;

import edu.wpi.first.wpilibj2.command.Command;
import com.redstorm509.alice2024.subsystems.Intake;
import com.redstorm509.alice2024.subsystems.drive.SwerveDrive;

public class AutoPickup extends Command {
	private final Intake m_Intake;
	private final SwerveDrive m_Swerve;

	public AutoPickup(Intake intake, SwerveDrive swerve) {
		m_Intake = intake;
		m_Swerve = swerve;

		addRequirements(m_Intake, m_Swerve);
	}

	@Override
	public void execute() {
	}
}
