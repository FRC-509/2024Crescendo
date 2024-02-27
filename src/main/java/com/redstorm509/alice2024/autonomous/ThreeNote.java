package com.redstorm509.alice2024.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.redstorm509.alice2024.subsystems.Intake;
import com.redstorm509.alice2024.subsystems.Shooter;
import com.redstorm509.alice2024.subsystems.drive.SwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ThreeNote extends SequentialCommandGroup {

	public ThreeNote(SwerveDrive swerve, Shooter shooter, Intake intake) {
		Pose2d startPose = new Pose2d(0.63, 6.62, Rotation2d.fromDegrees(59.47));
		Command paths = Commands.sequence(
				SwerveDrive.resetOdometryCmd(swerve, startPose),
				AutoBuilder.followPath(PathPlannerPath.fromPathFile("DriveToNote")),
				AutoBuilder.followPath(PathPlannerPath.fromPathFile("DriveToNoteRev")),
				AutoBuilder.followPath(PathPlannerPath.fromPathFile("DriveToSecondNote")),
				AutoBuilder.followPath(PathPlannerPath.fromPathFile("DriveToSecondNoteRev")),
				Commands.runOnce(() -> swerve.stopModules(), swerve));
		addCommands(paths);
	}
}
