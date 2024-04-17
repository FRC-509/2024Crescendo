package com.redstorm509.alice2024.autonomous.Actions;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.redstorm509.alice2024.Constants;
import com.redstorm509.alice2024.commands.SetHeading;
import com.redstorm509.alice2024.commands.SetPivot;
import com.redstorm509.alice2024.commands.autonomous.AutoShootMoreJank;
import com.redstorm509.alice2024.commands.autonomous.AutonomousIntakeNote;
import com.redstorm509.alice2024.subsystems.Arm;
import com.redstorm509.alice2024.subsystems.Indexer;
import com.redstorm509.alice2024.subsystems.Intake;
import com.redstorm509.alice2024.subsystems.Shooter;
import com.redstorm509.alice2024.subsystems.drive.SwerveDrive;
import com.redstorm509.alice2024.util.drivers.REVBlinkin;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DriveToAndShootNote2Paths extends SequentialCommandGroup {
	public DriveToAndShootNote2Paths(String pathToNote, String pathToShoot, double heading, double armPivot,
			SwerveDrive swerve, Arm arm, Shooter shooter, Indexer indexer, Intake intake, REVBlinkin lights) {
		Command paths = Commands.sequence(
				Commands.parallel(
						Commands.sequence(
								new SetPivot(arm, Constants.Arm.kMinPivot),
								AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathToNote)),
								Commands.runOnce(() -> swerve.stopModules(), swerve),
								Commands.waitUntil(() -> indexer.isNoteInsideRobot()),
								Commands.parallel(
										AutoBuilder
												.followPath(PathPlannerPath.fromPathFile(pathToShoot)),
										Commands.sequence(
												Commands.waitUntil(() -> indexer.isNoteInsideIndexer()),
												new SetPivot(arm, armPivot))),
								new DeferredCommand(
										() -> new SetHeading(swerve, swerve.jankFlipHeading(heading)),
										Set.of(swerve))),
						new AutonomousIntakeNote(intake, indexer, lights)),
				Commands.runOnce(() -> swerve.stopModules(), swerve),
				new AutoShootMoreJank(shooter, indexer));
		addCommands(paths);
	}

	public DriveToAndShootNote2Paths(String pathToNote, String pathToShoot, double armPivot, SwerveDrive swerve,
			Arm arm, Shooter shooter, Indexer indexer, Intake intake, REVBlinkin lights) {
		Command paths = Commands.sequence(
				Commands.parallel(
						Commands.sequence(
								new SetPivot(arm, Constants.Arm.kMinPivot),
								AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathToNote)),
								Commands.runOnce(() -> swerve.stopModules(), swerve),
								Commands.waitUntil(() -> indexer.isNoteInsideIndexer()),
								Commands.parallel(
										AutoBuilder
												.followPath(PathPlannerPath.fromPathFile(pathToShoot)),
										Commands.sequence(
												Commands.waitUntil(() -> indexer.hasNote()),
												new SetPivot(arm, armPivot)))),
						new AutonomousIntakeNote(intake, indexer, lights)),
				Commands.runOnce(() -> swerve.stopModules(), swerve),
				new AutoShootMoreJank(shooter, indexer));
		addCommands(paths);
	}
}
