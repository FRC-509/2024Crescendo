package com.redstorm509.alice2024.commands;

import com.redstorm509.alice2024.subsystems.Intake;
import com.redstorm509.alice2024.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeNote extends Command {
	private final Intake intake;
	private final Shooter shooter;

	public IntakeNote(Intake intake, Shooter shooter) {
		this.intake = intake;
		this.shooter = shooter;

		addRequirements(intake);
	}

	@Override
	public void execute() {
		if (shooter.indexerHasNote()) {
			end(true);
		}

		intake.intake(true);

	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean wasInterrupted) {
		intake.stop();
	}
}
