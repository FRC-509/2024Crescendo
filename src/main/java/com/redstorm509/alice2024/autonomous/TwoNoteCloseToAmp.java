package com.redstorm509.alice2024.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.redstorm509.alice2024.commands.AUTOPIVOTAHH;
import com.redstorm509.alice2024.commands.AutoShootJank;
import com.redstorm509.alice2024.commands.DefaultDriveCommand;
import com.redstorm509.alice2024.commands.IntakeNote;
import com.redstorm509.alice2024.commands.SetPivot;
import com.redstorm509.alice2024.commands.ShootNote;
import com.redstorm509.alice2024.subsystems.ArmIS;
import com.redstorm509.alice2024.subsystems.ArmIS;
import com.redstorm509.alice2024.subsystems.Indexer;
import com.redstorm509.alice2024.subsystems.Intake;
import com.redstorm509.alice2024.subsystems.Shooter;
import com.redstorm509.alice2024.subsystems.drive.SwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TwoNoteCloseToAmp extends SequentialCommandGroup {
	public TwoNoteCloseToAmp(SwerveDrive swerve, Shooter shooter, ArmIS arm, Indexer indexer, Intake intake) {
		Pose2d startPose = new Pose2d(0.72, 6.65, Rotation2d.fromDegrees(59.86));
		Command paths = Commands.sequence(
				new AutoShootJank(shooter, indexer),
				SwerveDrive.resetOdometryCmd(swerve, startPose),
				Commands.parallel(
						Commands.sequence(
								AutoBuilder.followPath(PathPlannerPath.fromPathFile("D2N_TwoNoteAmpSide")),

								Commands.runOnce(() -> swerve.setTargetHeading(SwerveDrive.jankFlipHeading(-33.83)),
										swerve),
								new DefaultDriveCommand(swerve, 0.0, 0.0, 0.0, true).withTimeout(0.5),

								Commands.runOnce(() -> swerve.stopModules(), swerve)),
						new IntakeNote(intake, indexer, arm)),
				new SetPivot(arm, -50),
				// new AUTOPIVOTAHH(arm, -48.976).withTimeout(3),
				new AutoShootJank(shooter, indexer));
		addCommands(paths);
	}
}
