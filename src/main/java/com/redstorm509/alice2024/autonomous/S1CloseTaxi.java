package com.redstorm509.alice2024.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.redstorm509.alice2024.commands.autonomous.AutoShootMoreJank;
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
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class S1CloseTaxi extends SequentialCommandGroup {
	public S1CloseTaxi(SwerveDrive swerve, Shooter shooter, Arm arm, Indexer indexer, Intake intake,
			REVBlinkin lights) {
		Pose2d startPose = new Pose2d(0.73, 4.47, Rotation2d.fromDegrees(-59.86));
		Command paths = Commands.sequence(
				shooter.startShooting(),
				swerve.resetOdometryCmd(startPose),
				new AutoShootMoreJank(shooter, indexer),
				new WaitCommand(1.0),
				AutoBuilder.followPath(PathPlannerPath.fromPathFile("TaxiSourceSide")),
				Commands.runOnce(() -> swerve.stopModules(), swerve),
				shooter.stopShooting());
		addCommands(paths);
	}
}
