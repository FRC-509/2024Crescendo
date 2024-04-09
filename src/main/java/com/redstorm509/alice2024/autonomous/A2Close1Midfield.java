package com.redstorm509.alice2024.autonomous;

import com.redstorm509.alice2024.autonomous.Actions.DriveToAndShootNote;
import com.redstorm509.alice2024.autonomous.Actions.DriveToAndShootNote2Paths;
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

public class A2Close1Midfield extends SequentialCommandGroup {
	public A2Close1Midfield(SwerveDrive swerve, Shooter shooter, Arm arm, Indexer indexer,
			Intake intake, REVBlinkin lights) {
		Pose2d startPose = new Pose2d(0.72, 6.65, Rotation2d.fromDegrees(59.86));
		Command paths = Commands.sequence(
				shooter.startShooting(),
				new AutoShootMoreJank(shooter, indexer),
				swerve.resetOdometryCmd(startPose),
				new DriveToAndShootNote("D2N_TwoNoteAmpSideHACK", 31.16, -34.453750, swerve, arm, shooter, indexer,
						intake,
						lights),
				new DriveToAndShootNote2Paths("DriveToThirdNoteFar", "DriveToThirdNoteFarRev", 38.56, -34.453750,
						swerve, arm, shooter, indexer, intake, lights),
				shooter.stopShooting());
		addCommands(paths);
	}
}
