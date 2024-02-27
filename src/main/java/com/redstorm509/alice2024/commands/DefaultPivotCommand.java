package com.redstorm509.alice2024.commands;

import java.util.function.DoubleSupplier;

import com.redstorm509.alice2024.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class DefaultPivotCommand extends Command {
	private final Shooter shooter;
	private final DoubleSupplier pivotOutputSup;

	public DefaultPivotCommand(Shooter shooter, DoubleSupplier pivotOutputSup) {
		this.shooter = shooter;
		this.pivotOutputSup = pivotOutputSup;
		addRequirements(shooter);
	}

	@Override
	public void execute() {
		shooter.setPivotOutput(pivotOutputSup.getAsDouble());
	}
}