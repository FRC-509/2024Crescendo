package com.redstorm509.alice2024.autonomous.AdaptiveBehavior;

import com.pathplanner.lib.commands.PathfindHolonomic;
import com.pathplanner.lib.path.PathConstraints;
import com.redstorm509.alice2024.commands.IntakeNote;
import com.redstorm509.alice2024.commands.autonomous.AutonomousAutoPickup;
import com.redstorm509.alice2024.commands.autonomous.FindNearestNote;
import com.redstorm509.alice2024.subsystems.Arm;
import com.redstorm509.alice2024.subsystems.Indexer;
import com.redstorm509.alice2024.subsystems.Intake;
import com.redstorm509.alice2024.subsystems.Shooter;
import com.redstorm509.alice2024.subsystems.drive.SwerveDrive;
import com.redstorm509.alice2024.subsystems.vision.Limelight;
import com.redstorm509.alice2024.util.drivers.REVBlinkin;
import com.redstorm509.alice2024.util.drivers.REVBlinkin.ColorCode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AdaptableBehavior extends SequentialCommandGroup {
	public AdaptableBehavior(Pose2d endPose, SwerveDrive swerve, Shooter shooter, Arm arm, Indexer indexer,
			Intake intake, Limelight intakeLL, Limelight armLL, REVBlinkin lights) {
		Command paths = Commands.sequence(
				new InstantCommand(() -> lights.setColor(ColorCode.ERROR), lights),
				new WaitCommand(0.2),
				new FindNearestNote(swerve),
				new AutonomousAutoPickup(swerve, intakeLL, intake, indexer, lights),
				Commands.parallel(
						new IntakeNote(intake, indexer, lights),
						new PathfindHolonomic(
								endPose,
								new PathConstraints(5.02, 5.02, Math.toRadians(660), Math.toRadians(660)),
								swerve::getEstimatedPose,
								swerve::getChassisSpeeds,
								swerve::setChassisSpeeds,
								swerve.getHolonomicPathFollowerConfig(),
								swerve)));
		addCommands(paths);
	}
}
