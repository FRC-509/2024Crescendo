package com.redstorm509.alice2024.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.redstorm509.alice2024.Constants;
import com.redstorm509.alice2024.commands.AutoShootJank;
import com.redstorm509.alice2024.commands.AutonomousIntakeNote;
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

public class TwoNoteCloseToAmp extends SequentialCommandGroup {
	public TwoNoteCloseToAmp(SwerveDrive swerve, Shooter shooter, ArmRS arm, Indexer indexer, Intake intake) {
		Pose2d startPose = new Pose2d(0.72, 6.65, Rotation2d.fromDegrees(59.86));
		Command paths = Commands.sequence(
			new AutonomousIntakeNote(intake, indexer),
			new SetPivot(arm, -50.5),
			new AutoShootJank(shooter, indexer),
			new SetPivot(arm, Constants.Arm.kMinPivot),
			Commands.runOnce(() -> indexer.setNoteless(), indexer),
				swerve.resetOdometryCmd(startPose),
				Commands.parallel(
					AutoBuilder.followPath(PathPlannerPath.fromPathFile("FD2N_TwoNoteAmpSide")),
					new AutonomousIntakeNote(intake, indexer)
				),
				Commands.runOnce(() -> swerve.setTargetHeading(swerve.jankFlipHeading(29.56)),	swerve),
				new DefaultDriveCommand(swerve, 0.0, 0.0, 0.0, true).withTimeout(0.5),
				Commands.runOnce(() -> swerve.stopModules(), swerve),
				new SetPivot(arm, -38.622),
				new AutoShootJank(shooter, indexer),
				new SetPivot(arm, Constants.Arm.kMinPivot),
				Commands.runOnce(() -> swerve.stopModules(), swerve),
				Commands.parallel(
					AutoBuilder.followPath(PathPlannerPath.fromPathFile("DriveToThirdNoteClose")),
					new AutonomousIntakeNote(intake, indexer)
				),
				Commands.runOnce(() -> swerve.setTargetHeading(swerve.jankFlipHeading(13.42)), swerve),
				new DefaultDriveCommand(swerve, 0.0, 0.0, 0.0, true).withTimeout(0.5),
				Commands.runOnce(() -> swerve.stopModules(), swerve),
				new SetPivot(arm, -38.232).withTimeout(1),
				new AutoShootJank(shooter, indexer),
				new SetPivot(arm, Constants.Arm.kMinPivot),
				Commands.parallel(
					AutoBuilder.followPath(PathPlannerPath.fromPathFile("DriveToFourthNoteClose")),
					new AutonomousIntakeNote(intake, indexer)
				),
				Commands.runOnce(() -> swerve.stopModules(), swerve),
				new AutoShootJank(shooter, indexer));
		addCommands(paths);
	}
}
