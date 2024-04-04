package com.redstorm509.alice2024.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.redstorm509.alice2024.Constants;
import com.redstorm509.alice2024.autonomous.Actions.DriveToAndShootNote;
import com.redstorm509.alice2024.commands.SetHeading;
import com.redstorm509.alice2024.commands.SetPivot;
import com.redstorm509.alice2024.commands.autonomous.AutoShootMoreJank;
import com.redstorm509.alice2024.commands.autonomous.AutonomousIntakeNote;
import com.redstorm509.alice2024.commands.autonomous.AutonomousShootEvenMoreJankButItsOk;
import com.redstorm509.alice2024.subsystems.Arm;
import com.redstorm509.alice2024.subsystems.Indexer;
import com.redstorm509.alice2024.subsystems.Intake;
import com.redstorm509.alice2024.subsystems.Shooter;
import com.redstorm509.alice2024.subsystems.drive.SwerveDrive;
import com.redstorm509.alice2024.util.drivers.REVBlinkin;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class A2Close1Midfield extends SequentialCommandGroup {
	public A2Close1Midfield(SwerveDrive swerve, Shooter shooter, Arm arm, Indexer indexer,
			Intake intake, REVBlinkin lights) {
		Pose2d startPose = new Pose2d(0.72, 6.65, Rotation2d.fromDegrees(59.86));
		Command paths = Commands.sequence(
				shooter.startShooting(),
				new AutonomousShootEvenMoreJankButItsOk(-Constants.Shooter.kSpeakerShootSpeed, shooter, indexer),
				swerve.resetOdometryCmd(startPose),
				new DriveToAndShootNote("D2N_TwoNoteAmpSide", 31.16, -34.453750, swerve, arm, shooter, indexer, intake,
						lights),
				new SetPivot(arm, Constants.Arm.kMinPivot),
				Commands.runOnce(() -> swerve.stopModules(), swerve),
				Commands.parallel(
						Commands.sequence(
								Commands.parallel(
										new SetPivot(arm, Constants.Arm.kMinPivot),
										AutoBuilder.followPath(PathPlannerPath.fromPathFile("DriveToThirdNoteFar"))),
								Commands.waitUntil(() -> indexer.isNoteInside())),
						new AutonomousIntakeNote(intake, indexer, lights)),
				Commands.runOnce(() -> swerve.stopModules(), swerve),
				Commands.parallel(
						AutoBuilder.followPath(PathPlannerPath.fromPathFile("DriveToThirdNoteFarRev")),
						new SetHeading(swerve, swerve.jankFlipHeading(24.2)),
						new SetPivot(arm, -24.05)),
				Commands.runOnce(() -> swerve.stopModules(), swerve),
				new AutoShootMoreJank(shooter, indexer),
				new DriveToAndShootNote("DriveToFourthNoteFar", 24.2, -24.05, swerve, arm, shooter, indexer, intake,
						lights),
				Commands.runOnce(() -> swerve.stopModules(), swerve),
				shooter.stopShooting());
		addCommands(paths);
	}
}
