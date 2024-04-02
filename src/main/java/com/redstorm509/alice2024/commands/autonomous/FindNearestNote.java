package com.redstorm509.alice2024.commands.autonomous;

import com.redstorm509.alice2024.subsystems.drive.SwerveDrive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class FindNearestNote extends Command {
	private SwerveDrive swerve;

	private double desiredHeading;

	public FindNearestNote(SwerveDrive swerve) {
		this.swerve = swerve;

		addRequirements(swerve);
	}

	@Override
	public void initialize() {
		Pose2d nearestNote = swerve.getEstimatedPose().nearest(swerve.field2d.getObject("Note").getPoses());
		// .relativeTo() is correct function ?
		desiredHeading = nearestNote.relativeTo(swerve.getEstimatedPose()).getRotation().getDegrees();

		swerve.setTargetHeading(desiredHeading);
	}

	@Override
	public boolean isFinished() {
		return MathUtil.isNear(desiredHeading, swerve.getYaw().getDegrees(), 0.5);// fix/tune
	}
}
