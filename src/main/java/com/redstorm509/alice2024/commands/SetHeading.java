package com.redstorm509.alice2024.commands;

import com.redstorm509.alice2024.subsystems.drive.SwerveDrive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class SetHeading extends Command {
	private SwerveDrive swerve;
	private double targetAngle;

	public SetHeading(SwerveDrive swerve, double targetAngle) {
		this.swerve = swerve;
		this.targetAngle = targetAngle;
		addRequirements(swerve);
	}

	@Override
	public void initialize() {
		swerve.setTargetHeading(targetAngle);
	}

	@Override
	public void execute() {
		swerve.drive(new Translation2d(), 0, true, false);
	}

	@Override
	public boolean isFinished() {
		return MathUtil.isNear(targetAngle, swerve.getYaw().getDegrees(), 0.25);
	}

	@Override
	public void end(boolean wasInterrupted) {
		swerve.stopModules();
	}
}
