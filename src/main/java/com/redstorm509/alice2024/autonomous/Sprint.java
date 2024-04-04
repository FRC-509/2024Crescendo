package com.redstorm509.alice2024.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.redstorm509.alice2024.Constants;
import com.redstorm509.alice2024.commands.SetPivot;
import com.redstorm509.alice2024.commands.autonomous.AutoShootMoreJank;
import com.redstorm509.alice2024.commands.autonomous.AutonomousIntakeNote;
import com.redstorm509.alice2024.commands.autonomous.AutonomousShootEvenMoreJankButItsOk;
import com.redstorm509.alice2024.subsystems.*;
import com.redstorm509.alice2024.subsystems.drive.SwerveDrive;
import com.redstorm509.alice2024.subsystems.vision.Limelight;
import com.redstorm509.alice2024.util.drivers.REVBlinkin;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Sprint extends SequentialCommandGroup {
	public Sprint(SwerveDrive swerve, Arm arm, Intake intake, Indexer indexer, Shooter shooter, Limelight shooterCamera,
			REVBlinkin lights) {
		Pose2d startPose = new Pose2d(1.42, 1.58, Rotation2d.fromDegrees(0));
		Command paths = Commands.sequence(
				shooter.startShooting(),
				swerve.resetOdometryCmd(startPose),
				Commands.parallel(
						new AutonomousIntakeNote(intake, indexer, lights),
						AutoBuilder.followPath(PathPlannerPath.fromPathFile("Sprint1"))),
				Commands.parallel(
						new SetPivot(arm, 0),
						AutoBuilder.followPath(PathPlannerPath.fromPathFile("Sprint2"))),
				Commands.runOnce(swerve::stopModules, swerve),
				new AutoShootMoreJank(shooter, indexer),
				Commands.parallel(
						Commands.sequence(
								new SetPivot(arm, Constants.Arm.kMinPivot),
								new AutonomousIntakeNote(intake, indexer, lights)),
						AutoBuilder.followPath(PathPlannerPath.fromPathFile("Sprint3"))),
				Commands.parallel(
						new SetPivot(arm, 0),
						AutoBuilder.followPath(PathPlannerPath.fromPathFile("Sprint4"))),
				Commands.runOnce(swerve::stopModules, swerve),
				new AutoShootMoreJank(shooter, indexer),
				Commands.parallel(
						Commands.sequence(
								new SetPivot(arm, Constants.Arm.kMinPivot),
								new AutonomousIntakeNote(intake, indexer, lights)),
						AutoBuilder.followPath(PathPlannerPath.fromPathFile("Sprint5"))),
				Commands.parallel(
						new SetPivot(arm, 0),
						AutoBuilder.followPath(PathPlannerPath.fromPathFile("Sprint6"))),
				Commands.runOnce(swerve::stopModules, swerve),
				new AutonomousShootEvenMoreJankButItsOk(-Constants.Shooter.kSpeakerShootSpeed, shooter, indexer),
				shooter.stopShooting());
		addCommands(paths);
	}
}