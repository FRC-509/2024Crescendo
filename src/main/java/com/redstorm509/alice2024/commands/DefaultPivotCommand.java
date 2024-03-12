package com.redstorm509.alice2024.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.redstorm509.alice2024.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.Command;

public class DefaultPivotCommand extends Command {
	private final Arm arm;
	private final DoubleSupplier pivotOutputSup;
	private final BooleanSupplier rawOutputMode;
	private boolean enteredRawOutputMode;

	public DefaultPivotCommand(Arm arm, DoubleSupplier pivotOutputSup, BooleanSupplier rawOutputMode) {
		this.arm = arm;
		this.pivotOutputSup = pivotOutputSup;
		this.rawOutputMode = rawOutputMode;
		addRequirements(arm);
	}

	@Override
	public void execute() {
		if (rawOutputMode.getAsBoolean()) {
			enteredRawOutputMode = true;
			arm.setPivotOpenLoop(pivotOutputSup.getAsDouble());
		} else if (!rawOutputMode.getAsBoolean() && enteredRawOutputMode) {
			enteredRawOutputMode = false;
			arm.setPivotDegrees(arm.getPivotDegrees());
		} else if (!rawOutputMode.getAsBoolean() && !enteredRawOutputMode) {
			arm.setPivotOutput(pivotOutputSup.getAsDouble());
		}
	}
}