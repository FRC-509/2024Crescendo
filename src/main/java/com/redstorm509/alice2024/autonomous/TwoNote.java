package com.redstorm509.alice2024.autonomous;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.GeometryUtil;
import com.redstorm509.alice2024.commands.IntakeNote;
import com.redstorm509.alice2024.commands.ShootNote;
import com.redstorm509.alice2024.subsystems.Intake;
import com.redstorm509.alice2024.subsystems.Shooter;
import com.redstorm509.alice2024.subsystems.drive.SwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TwoNote extends SequentialCommandGroup {
	private static Command resetOdometryCmd(SwerveDrive swerve, Pose2d pose) {
		return Commands.runOnce(
				() -> {
					boolean flip = false;
					Optional<Alliance> alliance = DriverStation.getAlliance();
					if (alliance.isPresent()) {
						flip = alliance.get() == DriverStation.Alliance.Red;
					}
					if (flip) {
						swerve.resetOdometry(GeometryUtil.flipFieldPose(pose));
					} else {
						swerve.resetOdometry(pose);
					}
				}, swerve);
	}

	public TwoNote(SwerveDrive swerve, Shooter shooter, Intake intake) {
		Pose2d startPose = new Pose2d(1.24, 5.56, Rotation2d.fromDegrees(0));
		Command paths = Commands.sequence(
				resetOdometryCmd(swerve, startPose),
				AutoBuilder.followPath(PathPlannerPath.fromPathFile("DriveToNoteShort")),
				AutoBuilder.followPath(PathPlannerPath.fromPathFile("DriveToNoteShortRev")),
				Commands.runOnce(() -> swerve.stopModules(), swerve));
		// addCommands(paths);

		addCommands(
				new SequentialCommandGroup(
						new ShootNote(shooter, 100.0, true).withTimeout(2),
						new ParallelCommandGroup(
								paths,
								new IntakeNote(intake, shooter).withTimeout(2)),
						new ShootNote(shooter, 100.0, true).withTimeout(2)));
	}
}
