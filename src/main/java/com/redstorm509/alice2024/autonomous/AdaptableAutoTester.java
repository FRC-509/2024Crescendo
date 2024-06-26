package com.redstorm509.alice2024.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.redstorm509.alice2024.autonomous.AdaptiveBehavior.AdaptableBehavior;
import com.redstorm509.alice2024.commands.autonomous.AutoShootMoreJank;
import com.redstorm509.alice2024.commands.autonomous.AutonomousIntakeNote;
import com.redstorm509.alice2024.subsystems.Arm;
import com.redstorm509.alice2024.subsystems.Indexer;
import com.redstorm509.alice2024.subsystems.Indexer.IndexerState;
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

public class AdaptableAutoTester extends SequentialCommandGroup {
	public AdaptableAutoTester(SwerveDrive swerve, Shooter shooter, Arm arm, Indexer indexer, Intake intake,
			Limelight intakeLL, Limelight armLL, REVBlinkin lights) {
		// PATHS ARE NOT REAL :(
		Pose2d startPose = new Pose2d(0.73, 4.47, Rotation2d.fromDegrees(-59.86));
		Command paths = Commands.sequence(
				shooter.startShooting(),
				AutoBuilder.followPath(PathPlannerPath.fromPathFile("PathToNote")),
				Commands.either(
						AutoBuilder.followPath(PathPlannerPath.fromPathFile("PathBack")),
						new AdaptableBehavior(startPose, swerve, shooter, arm, indexer, intake, intakeLL, armLL,
								lights),
						() -> indexer.indexingNoteState != IndexerState.Noteless),
				swerve.resetOdometryCmd(startPose),
				new AutonomousIntakeNote(intake, indexer, lights),
				new AutoShootMoreJank(shooter, indexer),
				shooter.stopShooting());
		addCommands(paths);
	}
}
