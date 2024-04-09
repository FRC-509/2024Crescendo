package com.redstorm509.alice2024.autonomous;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.redstorm509.alice2024.Constants;
import com.redstorm509.alice2024.autonomous.Actions.DriveToAndShootNote2Paths;
import com.redstorm509.alice2024.commands.SetHeading;
import com.redstorm509.alice2024.commands.SetPivot;
import com.redstorm509.alice2024.commands.autonomous.AutoShootMoreJank;
import com.redstorm509.alice2024.commands.autonomous.AutonomousIntakeNote;
import com.redstorm509.alice2024.subsystems.*;
import com.redstorm509.alice2024.subsystems.drive.SwerveDrive;
import com.redstorm509.alice2024.subsystems.vision.Limelight;
import com.redstorm509.alice2024.util.drivers.REVBlinkin;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Sprint extends SequentialCommandGroup {
	public Sprint(SwerveDrive swerve, Arm arm, Intake intake, Indexer indexer, Shooter shooter, Limelight shooterCamera,
			REVBlinkin lights) {
		Pose2d startPose = new Pose2d(1.42, 1.58, Rotation2d.fromDegrees(0));
		Command paths = Commands.sequence(
				shooter.startShooting(),
				swerve.resetOdometryCmd(startPose),
				new DriveToAndShootNote2Paths("Sprint1", "Sprint2", -25.49, -26.57 - 10, swerve, arm, shooter, indexer,
						intake,
						lights),
				new DriveToAndShootNote2Paths("Sprint3", "Sprint4", -25.49, -26.57 - 10, swerve, arm, shooter, indexer,
						intake,
						lights),
				Commands.sequence(
						Commands.parallel(
								Commands.sequence(
										new SetPivot(arm, Constants.Arm.kMinPivot),
										AutoBuilder.followPath(PathPlannerPath.fromPathFile("Sprint5")),
										Commands.runOnce(() -> swerve.stopModules(), swerve),
										Commands.waitUntil(() -> indexer.isNoteInside()),
										AutoBuilder.followPath(PathPlannerPath.fromPathFile("Sprint6")),
										Commands.waitUntil(() -> indexer.hasNote()),
										new SetPivot(arm, -25.49),
										new DeferredCommand(
												() -> new SetHeading(swerve, swerve.jankFlipHeading(-26.57 + 10)),
												Set.of(swerve))),
								new AutonomousIntakeNote(intake, indexer, lights)),
						Commands.runOnce(() -> swerve.stopModules(), swerve),
						new AutoShootMoreJank(shooter, indexer)),
				shooter.stopShooting());
		addCommands(paths);
	}
}