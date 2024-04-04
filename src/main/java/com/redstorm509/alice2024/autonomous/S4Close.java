package com.redstorm509.alice2024.autonomous;

import com.redstorm509.alice2024.Constants;
import com.redstorm509.alice2024.autonomous.Actions.DriveToAndShootNote;
import com.redstorm509.alice2024.commands.SetPivot;
import com.redstorm509.alice2024.commands.autonomous.AutonomousShootEvenMoreJankButItsOk;
import com.redstorm509.alice2024.subsystems.Arm;
import com.redstorm509.alice2024.subsystems.Indexer;
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

public class S4Close extends SequentialCommandGroup {
	public S4Close(SwerveDrive swerve, Shooter shooter, Arm arm, Indexer indexer, Intake intake,
			Limelight shooterCamera, REVBlinkin lights) {
		Pose2d startPose = new Pose2d(0.72, 4.43, Rotation2d.fromDegrees(-59.86));
		Command paths = Commands.sequence(
				shooter.startShooting(),
				new AutonomousShootEvenMoreJankButItsOk(Constants.Shooter.kSpeakerShootSpeed, shooter, indexer),
				swerve.resetOdometryCmd(startPose),
				new DriveToAndShootNote("D2N_TwoNoteSourceSide", -31.16, -34.453750, swerve, arm, shooter, indexer,
						intake,
						lights),
				new DriveToAndShootNote("SDriveToThirdNoteClose", 0.0, -37.441406, swerve, arm, shooter, indexer,
						intake,
						lights),
				new DriveToAndShootNote("SDriveToFourthNoteClose", 27.475, -35.683594, swerve, arm, shooter, indexer,
						intake,
						lights),
				new SetPivot(arm, Constants.Arm.kMinPivot),
				Commands.runOnce(() -> swerve.stopModules(), swerve),
				shooter.stopShooting());
		addCommands(paths);
	}
}
