package com.redstorm509.alice2024.autonomous.Actions;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.redstorm509.alice2024.subsystems.Intake;
import com.redstorm509.alice2024.subsystems.Shooter;
import com.redstorm509.alice2024.commands.SetPivot;
import com.redstorm509.alice2024.commands.autonomous.AutonomousIntakeNote;
import com.redstorm509.alice2024.commands.autonomous.AutonomousRemoveNoteShooter;
import com.redstorm509.alice2024.commands.autonomous.AutonomousShootEvenMoreJankButItsOk;
import com.redstorm509.alice2024.subsystems.Arm;
import com.redstorm509.alice2024.subsystems.Indexer;
import com.redstorm509.alice2024.subsystems.drive.SwerveDrive;
import com.redstorm509.alice2024.util.drivers.REVBlinkin;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class DriveAndShootWhileMoving extends SequentialCommandGroup {
	public DriveAndShootWhileMoving(String pathName, double timeToShoot, double armPivot, SwerveDrive swerve, Arm arm,
			Shooter shooter, Indexer indexer,
			Intake intake, REVBlinkin lights) {
		Command paths = Commands.sequence(
				Commands.parallel(
						AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathName)),
						Commands.sequence(
								Commands.waitUntil(() -> indexer.isNoteInsideIndexer()),
								new SetPivot(arm, armPivot)),
						Commands.sequence(
								new WaitCommand(timeToShoot),
								Commands.either(
										new AutonomousShootEvenMoreJankButItsOk(armPivot, shooter, indexer),
										new AutonomousRemoveNoteShooter(shooter, indexer),
										() -> MathUtil.isNear(armPivot, arm.getPivotDegrees(), 4.0))),
						new AutonomousIntakeNote(intake, indexer, lights)),
				Commands.runOnce(() -> swerve.stopModules(), swerve));
		addCommands(paths);
	}
}
