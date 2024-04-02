package com.redstorm509.alice2024.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.redstorm509.alice2024.Constants;
import com.redstorm509.alice2024.commands.AutoAlign;
import com.redstorm509.alice2024.commands.DefaultDriveCommand;
import com.redstorm509.alice2024.commands.SetHeading;
import com.redstorm509.alice2024.commands.SetPivot;
import com.redstorm509.alice2024.commands.autonomous.AutoShootMoreJank;
import com.redstorm509.alice2024.commands.autonomous.AutonomousIntakeNote;
import com.redstorm509.alice2024.subsystems.Arm;
import com.redstorm509.alice2024.subsystems.Indexer;
import com.redstorm509.alice2024.subsystems.Intake;
import com.redstorm509.alice2024.subsystems.Shooter;
import com.redstorm509.alice2024.subsystems.drive.SwerveDrive;
import com.redstorm509.alice2024.subsystems.vision.Limelight;
import com.redstorm509.alice2024.util.drivers.REVBlinkin;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class A4Close extends SequentialCommandGroup {
	public A4Close(SwerveDrive swerve, Shooter shooter, Arm arm, Indexer indexer, Intake intake,
			Limelight shooterCamera, REVBlinkin lights) {
		Pose2d startPose = new Pose2d(0.72, 6.65, Rotation2d.fromDegrees(59.86));
		Command paths = Commands.sequence(
				shooter.startShooting(),
				new AutoShootMoreJank(shooter, indexer),
				swerve.resetOdometryCmd(startPose),
				Commands.parallel(
						AutoBuilder.followPath(PathPlannerPath.fromPathFile("FD2N_TwoNoteAmpSide")),
						new AutonomousIntakeNote(intake, indexer, lights)),
				Commands.runOnce(() -> swerve.stopModules(), swerve),
				Commands.parallel(
						new SetHeading(swerve, swerve.jankFlipHeading(31.16)),
						new SetPivot(arm, -34.453750)),
				new AutoShootMoreJank(shooter, indexer),
				Commands.parallel(
						new SetPivot(arm, Constants.Arm.kMinPivot),
						AutoBuilder.followPath(PathPlannerPath.fromPathFile("DriveToThirdNoteClose")),
						new AutonomousIntakeNote(intake, indexer, lights)),
				Commands.parallel(
						new SetHeading(swerve, swerve.jankFlipHeading(0)),
						new SetPivot(arm, -37.441406)),
				new AutoShootMoreJank(shooter, indexer),
				Commands.parallel(
						new SetPivot(arm, Constants.Arm.kMinPivot),
						AutoBuilder.followPath(PathPlannerPath.fromPathFile("DriveToFourthNoteClose")),
						new AutonomousIntakeNote(intake, indexer, lights)),
				Commands.parallel(
						new SetHeading(swerve, swerve.jankFlipHeading(-27.475)),
						new SetPivot(arm, -35.683594)),
				new AutoShootMoreJank(shooter, indexer),
				new SetPivot(arm, Constants.Arm.kMinPivot),
				Commands.runOnce(() -> swerve.stopModules(), swerve)
		/*-
		new AutoShootMoreJank(shooter, indexer),
		new SetPivot(arm, Constants.Arm.kMinPivot),
		Commands.parallel(
				AutoBuilder.followPath(PathPlannerPath.fromPathFile("DriveToThirdNoteClose")),
				new AutonomousIntakeNote(intake, indexer, lights)),
		Commands.runOnce(() -> swerve.stopModules(), swerve),
		new AutoAlign(swerve, arm, shooterCamera, lights),
		new AutoShootMoreJank(shooter, indexer),
		new SetPivot(arm, Constants.Arm.kMinPivot),
		Commands.parallel(
				AutoBuilder.followPath(PathPlannerPath.fromPathFile("DriveToFourthNoteClose")),
				new AutonomousIntakeNote(intake, indexer, lights)),
		Commands.runOnce(() -> swerve.stopModules(), swerve),
		new AutoAlign(swerve, arm, shooterCamera, lights),
		new AutoShootMoreJank(shooter, indexer),
		shooter.stopShooting()
		 */);
		addCommands(paths);
	}
}
