package com.redstorm509.alice2024.commands;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.redstorm509.alice2024.Constants;
import com.redstorm509.alice2024.subsystems.Shooter;
import com.redstorm509.alice2024.subsystems.drive.SwerveDrive;
import com.redstorm509.alice2024.subsystems.vision.Limelight;

public class AutoAmp extends Command {

	private SwerveDrive swerve;
	private Shooter shooter;
	private DoubleSupplier translationXSupplier;
	private DoubleSupplier translationYSupplier;
	private DoubleSupplier rotationSupplier;
	private Limelight limelight;
	private double rot;
	private double ampTagID;

	public AutoAmp(
			SwerveDrive swerve,
			Shooter shooter,
			DoubleSupplier translationXSupplier,
			DoubleSupplier translationYSupplier,
			DoubleSupplier rotationSupplier,
			Limelight limelight) {
		this.swerve = swerve;
		this.shooter = shooter;
		this.translationXSupplier = translationXSupplier;
		this.translationYSupplier = translationYSupplier;
		this.rotationSupplier = rotationSupplier;
		this.limelight = limelight;

		addRequirements(swerve, shooter);
	}

	@Override
	public void initialize() {
		limelight.setPipelineIndex(Constants.Vision.AprilTagPipeline);

		limelight.setLEDMode_ForceOn();

		if (!limelight.getTV()) {
			end(true);
		}

		Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
		if (alliance.isPresent()) {
			if (alliance.get() == DriverStation.Alliance.Blue) {
				ampTagID = Constants.AprilTags.Amp.BlueAllianceTagID;
			} else {
				ampTagID = Constants.AprilTags.Amp.RedAllianceTagID;
			}
		}
	}

	@Override
	public void execute() {
		if (limelight.getTV() && limelight.getFiducialID() == ampTagID) {
			rot = Math.toRadians(limelight.getTX() * 1.5); // tune this
			limelight.setLEDMode_ForceBlink();
		} else {
			rot = rotationSupplier.getAsDouble();
			limelight.setLEDMode_ForceOn();
		}

		swerve.drive(
				new Translation2d(translationXSupplier.getAsDouble(), translationYSupplier.getAsDouble())
						.times(Constants.kMaxSpeed),
				rot,
				true,
				false);
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean wasInterrupted) {
		limelight.setLEDMode_ForceOff();
		swerve.drive(new Translation2d(0, 0), 0, true, false);
	}

}
