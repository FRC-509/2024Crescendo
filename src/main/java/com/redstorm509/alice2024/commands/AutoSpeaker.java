package com.redstorm509.alice2024.commands;

import edu.wpi.first.wpilibj2.command.Command;
import com.redstorm509.alice2024.subsystems.Shooter;
import com.redstorm509.alice2024.subsystems.drive.SwerveDrive;

public class AutoSpeaker extends Command {
	private final Shooter m_Shooter;
	private final SwerveDrive m_Swerve;

	public AutoSpeaker(Shooter shooter, SwerveDrive swerve) {
		m_Shooter = shooter;
		m_Swerve = swerve;

		addRequirements(m_Shooter, m_Swerve);
	}

	@Override
	public void execute() {
	}
}