package com.redstorm509.alice2024.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.redstorm509.alice2024.commands.IntakeNote;
import com.redstorm509.alice2024.commands.ShootNote;
import com.redstorm509.alice2024.subsystems.Intake;
import com.redstorm509.alice2024.subsystems.Shooter;
import com.redstorm509.alice2024.subsystems.drive.SwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TwoNote extends SequentialCommandGroup {
	public TwoNote(SwerveDrive swerve, Shooter shooter, Intake intake) {
		addCommands(
				Commands.runOnce(() -> swerve.resetOdometry(new Pose2d(1.24, 5.56, Rotation2d.fromDegrees(0))), swerve),
				new SequentialCommandGroup(
						// new ShootNote(shooter, 100.0).withTimeout(2),
						// new ParallelCommandGroup(
						// new SequentialCommandGroup(
						AutoBuilder.followPath(PathPlannerPath.fromPathFile("DriveToNoteShort")),
						AutoBuilder.followPath(PathPlannerPath.fromPathFile("DriveToNoteShortRev")),
						new InstantCommand(() -> swerve.stopModules(), swerve))
		// new IntakeNote(intake, shooter).withTimeout(1)),
		// new ShootNote(shooter, 100.0).withTimeout(2))

		);
	}
}
