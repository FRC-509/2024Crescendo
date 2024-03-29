package com.redstorm509.alice2024.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.redstorm509.alice2024.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.Command;

public class DefaultPivotCommand extends Command {
	private final Arm arm;
	private final DoubleSupplier pivotOutputSup;

	public DefaultPivotCommand(Arm arm, DoubleSupplier pivotOutputSup, BooleanSupplier rawOutputMode) {
		this.arm = arm;
		this.pivotOutputSup = pivotOutputSup;
		addRequirements(arm);
	}

	@Override
	public void execute() {
		arm.setPivotOutput(pivotOutputSup.getAsDouble());
		/*-
		if (rawOutputMode.getAsBoolean()) {
			enteredRawOutputMode = true;
			// System.out.println("Doing Raw Output");
			arm.setPivotOpenLoop(pivotOutputSup.getAsDouble());
		} else if (!rawOutputMode.getAsBoolean() && enteredRawOutputMode) {
			// System.out.println("Going back to normal");
			enteredRawOutputMode = false;
			arm.setPivotDegrees(arm.getPivotDegrees());
		} else if (!rawOutputMode.getAsBoolean() && !enteredRawOutputMode) {
			// System.out.println("Being normal");
			arm.setPivotOutput(pivotOutputSup.getAsDouble());
		}
		 */
	}
}