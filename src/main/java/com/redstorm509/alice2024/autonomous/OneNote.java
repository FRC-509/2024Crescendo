package com.redstorm509.alice2024.autonomous;

import com.redstorm509.alice2024.commands.IntakeNote;
import com.redstorm509.alice2024.commands.autonomous.AutoShootJank;
import com.redstorm509.alice2024.subsystems.ArmRS;
import com.redstorm509.alice2024.subsystems.Indexer;
import com.redstorm509.alice2024.subsystems.Intake;
import com.redstorm509.alice2024.subsystems.Shooter;
import com.redstorm509.alice2024.subsystems.drive.SwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class OneNote extends SequentialCommandGroup {
	public OneNote(SwerveDrive swerve, Shooter shooter, ArmRS arm, Indexer indexer, Intake intake) {
		Pose2d startPose = new Pose2d(0.73, 4.47, Rotation2d.fromDegrees(-59.86));
		Command paths = Commands.sequence(swerve.resetOdometryCmd(startPose), new IntakeNote(intake, indexer),
				new AutoShootJank(shooter, indexer));
		addCommands(paths);
	}
}
