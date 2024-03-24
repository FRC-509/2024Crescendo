package com.redstorm509.alice2024.autonomous;

import com.ctre.phoenix6.controls.VoltageOut;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.redstorm509.alice2024.Constants;
import com.redstorm509.alice2024.commands.SetPivot;
import com.redstorm509.alice2024.commands.autonomous.AutoShootJank;
import com.redstorm509.alice2024.commands.autonomous.AutoShootMoreJank;
import com.redstorm509.alice2024.commands.autonomous.AutonomousIntakeNote;
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

public class ThreeNoteAmpSideDriveBack extends SequentialCommandGroup {
	public ThreeNoteAmpSideDriveBack(SwerveDrive swerve, Shooter shooter, ArmRS arm, Indexer indexer,
			Intake intake) {
		Pose2d startPose = new Pose2d(0.72, 6.65, Rotation2d.fromDegrees(59.86));
		Command paths = Commands.sequence(
				Commands.runOnce(() -> shooter.setShooterVelocity(-Constants.Shooter.kTargetSpeed), shooter),
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
						AutoBuilder.followPath(PathPlannerPath.fromPathFile("JankAutoPart3")),
						new AutonomousIntakeNote(intake, indexer)),
				AutoBuilder.followPath(PathPlannerPath.fromPathFile("JankAutoPart4")),
				Commands.runOnce(() -> swerve.stopModules(), swerve),
				new AutoShootMoreJank(shooter, indexer),
				Commands.runOnce(() -> shooter.shooterLeader.setControl(new VoltageOut(0)), shooter),
				Commands.parallel(
						AutoBuilder.followPath(PathPlannerPath.fromPathFile("JankAutoPart5")),
						new AutonomousIntakeNote(intake, indexer)));
		addCommands(paths);
	}

}