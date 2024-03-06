package com.redstorm509.alice2024.commands;

import java.util.function.DoubleSupplier;

import com.redstorm509.alice2024.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.Command;

public class DefaultPivotCommand extends Command {
	private final Arm arm;
	private final DoubleSupplier pivotOutputSup;

	public DefaultPivotCommand(Arm arm, DoubleSupplier pivotOutputSup) {
		this.arm = arm;
		this.pivotOutputSup = pivotOutputSup;
		addRequirements(arm);
	}

	@Override
	public void execute() {
		arm.setPivotOutput(pivotOutputSup.getAsDouble());
	}
}