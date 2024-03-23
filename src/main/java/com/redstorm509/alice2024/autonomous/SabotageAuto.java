package com.redstorm509.alice2024.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.redstorm509.alice2024.subsystems.drive.SwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SabotageAuto extends SequentialCommandGroup {
	public SabotageAuto(SwerveDrive swerve) {
		Pose2d startPose = new Pose2d(0.46, 4.12, Rotation2d.fromDegrees(0));
		addCommands(
				swerve.resetOdometryCmd(startPose),
				AutoBuilder.followPath(PathPlannerPath.fromPathFile("Sabotage")),
				Commands.runOnce(() -> swerve.stopModules(), swerve));
	}
}
