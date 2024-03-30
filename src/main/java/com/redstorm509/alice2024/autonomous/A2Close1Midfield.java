package com.redstorm509.alice2024.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.redstorm509.alice2024.Constants;
import com.redstorm509.alice2024.commands.SetPivot;
import com.redstorm509.alice2024.commands.autonomous.AutoShootMoreJank;
import com.redstorm509.alice2024.commands.autonomous.AutonomousIntakeNote;
import com.redstorm509.alice2024.subsystems.Arm;
import com.redstorm509.alice2024.subsystems.Indexer;
import com.redstorm509.alice2024.subsystems.Intake;
import com.redstorm509.alice2024.subsystems.Shooter;
import com.redstorm509.alice2024.subsystems.drive.SwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class A2Close1Midfield extends SequentialCommandGroup {
	public A2Close1Midfield(SwerveDrive swerve, Shooter shooter, Arm arm, Indexer indexer,
			Intake intake) {
		Pose2d startPose = new Pose2d(0.72, 6.65, Rotation2d.fromDegrees(59.86));
		Command paths = Commands.sequence(
				shooter.startShooting(),
				new AutonomousIntakeNote(intake, indexer),
				new AutoShootMoreJank(shooter, indexer),
				new SetPivot(arm, Constants.Arm.kMinPivot),
				swerve.resetOdometryCmd(startPose),
				Commands.parallel(
						AutoBuilder.followPath(PathPlannerPath.fromPathFile("FD2N_TwoNoteAmpSide")),
						new AutonomousIntakeNote(intake, indexer)),
				AutoBuilder.followPath(PathPlannerPath.fromPathFile("JankAutoPart2")),
				Commands.runOnce(() -> swerve.stopModules(), swerve),
				new AutoShootMoreJank(shooter, indexer),
				Commands.parallel(
						AutoBuilder.followPath(PathPlannerPath.fromPathFile("JankAutoPart5")),
						new AutonomousIntakeNote(intake, indexer)),
				AutoBuilder.followPath(PathPlannerPath.fromPathFile("JankAutoPart6")),
				Commands.runOnce(() -> swerve.stopModules(), swerve),
				new AutoShootMoreJank(shooter, indexer),
				shooter.stopShooting());
		addCommands(paths);
	}
}
