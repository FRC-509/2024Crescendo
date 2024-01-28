package com.redstorm509.alice2024.commands;

import com.redstorm509.alice2024.Constants;
import com.redstorm509.alice2024.subsystems.drive.SwerveDrive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveCommand extends Command {
	private final SwerveDrive swerve;
	private final DoubleSupplier translationXSupplier;
	private final DoubleSupplier translationYSupplier;
	private final DoubleSupplier rotationSupplier;
	private final BooleanSupplier fieldRelativeSupplier;

	public DriveCommand(SwerveDrive swerve, DoubleSupplier xSupplier, DoubleSupplier ySupplier,
			DoubleSupplier omegaSupplier, BooleanSupplier fieldRelativeSupplier) {
		this.swerve = swerve;
		this.translationXSupplier = xSupplier;
		this.translationYSupplier = ySupplier;
		this.rotationSupplier = omegaSupplier;
		this.fieldRelativeSupplier = fieldRelativeSupplier;

		addRequirements(swerve);
	}

	@Override
	public void execute() {
		// evaluate created members for x,y,theta, run drive command
		Translation2d trans = new Translation2d(translationXSupplier.getAsDouble(), translationYSupplier.getAsDouble())
				.times(Constants.kMaxSpeed);
		swerve.drive(trans, rotationSupplier.getAsDouble() * Constants.kMaxAngularVelocity,
				fieldRelativeSupplier.getAsBoolean(), false);
	}
}