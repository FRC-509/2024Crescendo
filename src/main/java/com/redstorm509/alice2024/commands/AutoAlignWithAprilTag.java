package com.redstorm509.alice2024.commands;

import java.util.function.DoubleSupplier;

import com.redstorm509.alice2024.Constants;
import com.redstorm509.alice2024.subsystems.drive.SwerveDrive;
import com.redstorm509.alice2024.subsystems.vision.Limelight;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoAlignWithAprilTag extends Command {

	private SwerveDrive swerve;
	private DoubleSupplier translationXSupplier;
	private DoubleSupplier translationYSupplier;
	private DoubleSupplier rotationSupplier;
	private Limelight limelight;
	private double targetTagID;

	private Translation2d targetPosition;
	private boolean isTeleop;

	private double rot;

	public AutoAlignWithAprilTag(
			SwerveDrive swerve,
			DoubleSupplier translationXSupplier,
			DoubleSupplier translationYSupplier,
			DoubleSupplier rotationSupplier,
			Limelight limelight,
			double targetTagID) {
		this.swerve = swerve;
		this.translationXSupplier = translationXSupplier;
		this.translationYSupplier = translationYSupplier;
		this.rotationSupplier = rotationSupplier;
		this.limelight = limelight;
		this.targetTagID = targetTagID;
		isTeleop = false;

		addRequirements(swerve);
	}

	public AutoAlignWithAprilTag(
			SwerveDrive swerve,
			Translation2d targetPosition,
			Limelight limelight,
			double targetTagID) {
		this.swerve = swerve;
		this.targetPosition = targetPosition;
		this.limelight = limelight;
		this.targetTagID = targetTagID;
		isTeleop = false;

		addRequirements(swerve);
	}

	@Override
	public void initialize() {
		limelight.setPipelineIndex(Constants.Vision.AprilTagPipeline);

		limelight.setLEDMode_ForceOn();

		if (!limelight.getTV()) {
			end(true);
		}
	}

	@Override
	public void execute() {
		if (limelight.getTV()) {
			rot = Math.toRadians(limelight.getTX() * 1.5); // tune this
			limelight.setLEDMode_ForceBlink();
		} else {
			rot = rotationSupplier.getAsDouble();
			limelight.setLEDMode_ForceOn();
		}

		if (isTeleop) {
			swerve.drive(
					new Translation2d(translationXSupplier.getAsDouble(), translationYSupplier.getAsDouble())
							.times(Constants.kMaxSpeed),
					rot,
					true,
					false);
		} else {
			swerve.drive(
					targetPosition,
					rot,
					true,
					false);
		}
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
