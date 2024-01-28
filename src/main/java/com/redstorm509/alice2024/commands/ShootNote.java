package com.redstorm509.alice2024.commands;

import com.redstorm509.alice2024.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootNote extends Command {
	private final Shooter m_ShooterSubsystem;

	public ShootNote(Shooter shooter) {
		m_ShooterSubsystem = shooter;
		addRequirements(m_ShooterSubsystem);
	}

	@Override
	public void execute() {
	}
}
