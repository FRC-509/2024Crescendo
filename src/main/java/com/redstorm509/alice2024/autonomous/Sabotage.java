package com.redstorm509.alice2024.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.redstorm509.alice2024.subsystems.*;
import com.redstorm509.alice2024.subsystems.drive.SwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Sabotage extends SequentialCommandGroup {
	public Sabotage(SwerveDrive swerve, Intake intake, Indexer indexer, Shooter shooter) {
		Pose2d startPose = new Pose2d(0.46, 4.12, Rotation2d.fromDegrees(0));
		addCommands(
				swerve.resetOdometryCmd(startPose),
				Commands.runOnce(() -> intake.intake(true), intake),
				Commands.runOnce(() -> indexer.rawIndexer(-1), indexer),
				Commands.runOnce(() -> shooter.setShooterVelocity(-7.5), shooter),

				AutoBuilder.followPath(PathPlannerPath.fromPathFile("Sabotage")),
				Commands.runOnce(() -> swerve.stopModules(), swerve),
				new WaitCommand(0.5),
				Commands.runOnce(() -> intake.stop(), intake),
				Commands.runOnce(() -> indexer.rawIndexer(0), indexer),
				shooter.stopShooting());
	}
}
