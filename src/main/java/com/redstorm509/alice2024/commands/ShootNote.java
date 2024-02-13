package com.redstorm509.alice2024.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.redstorm509.alice2024.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootNote extends Command {
	// change to use shooterMath once tested

	private final Shooter shooter;
	private final DoubleSupplier rotationSupplier;
	private final BooleanSupplier rightBumperSupplier;
	private final BooleanSupplier leftBumperSupplier;

	public ShootNote(Shooter shooter,
			DoubleSupplier rotationSupplier,
			BooleanSupplier rightBumperSupplier,
			BooleanSupplier leftBumperSupplier) {
		this.shooter = shooter;
		this.rotationSupplier = rotationSupplier;
		this.rightBumperSupplier = rightBumperSupplier;
		this.leftBumperSupplier = leftBumperSupplier;

		addRequirements(shooter);
	}

	@Override
	public void execute() {
		if (rightBumperSupplier.getAsBoolean()) {
			shooter.rawShootNote(0.5); // change to other value
		} else if (leftBumperSupplier.getAsBoolean()) {
			shooter.rawShootNote(-0.25); // comment out if necissary
		}
		shooter.setPivotOutput(rotationSupplier.getAsDouble());
	}
}
