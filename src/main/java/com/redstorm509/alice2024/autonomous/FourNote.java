package com.redstorm509.alice2024.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.redstorm509.alice2024.commands.AutoShootJank;
import com.redstorm509.alice2024.commands.DefaultDriveCommand;
import com.redstorm509.alice2024.commands.IntakeNote;
import com.redstorm509.alice2024.commands.SetPivot;
import com.redstorm509.alice2024.subsystems.ArmRS;
import com.redstorm509.alice2024.subsystems.ArmRS;
import com.redstorm509.alice2024.subsystems.Indexer;
import com.redstorm509.alice2024.subsystems.Intake;
import com.redstorm509.alice2024.subsystems.Shooter;
import com.redstorm509.alice2024.subsystems.drive.SwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FourNote extends SequentialCommandGroup {
	public FourNote(SwerveDrive swerve, Shooter shooter, ArmRS arm, Indexer indexer, Intake intake) {
		Pose2d startPose = new Pose2d(0.72, 6.65, Rotation2d.fromDegrees(59.86));
		Command paths = Commands.sequence(
			new IntakeNote(intake, indexer),
			new AutoShootJank(shooter, indexer),
				SwerveDrive.resetOdometryCmd(swerve, startPose),
				Commands.parallel(
					AutoBuilder.followPath(PathPlannerPath.fromPathFile("FD2N_TwoNoteAmpSide")),
					new IntakeNote(intake, indexer)
				),
				Commands.runOnce(() -> swerve.setTargetHeading(SwerveDrive.jankFlipHeading(27)),	swerve),
				new DefaultDriveCommand(swerve, 0.0, 0.0, 0.0, true).withTimeout(0.5),
				Commands.runOnce(() -> swerve.stopModules(), swerve),
				new AutoShootJank(shooter, indexer),
				Commands.parallel(
					AutoBuilder.followPath(PathPlannerPath.fromPathFile("DriveToSecondNote")),
					new IntakeNote(intake, indexer)
				),
				AutoBuilder.followPath(PathPlannerPath.fromPathFile("DriveToSecondNoteRev")),
				new AutoShootJank(shooter, indexer),
				Commands.parallel(
					AutoBuilder.followPath(PathPlannerPath.fromPathFile("DriveToThirdNote")),
					new IntakeNote(intake, indexer)
				),
				AutoBuilder.followPath(PathPlannerPath.fromPathFile("DriveToThirdNoteRev")),
				new AutoShootJank(shooter, indexer),
				Commands.runOnce(() -> swerve.stopModules(), swerve));
		addCommands(paths);
	}
}
