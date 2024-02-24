package com.redstorm509.alice2024.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;

import com.redstorm509.alice2024.Constants;
import com.redstorm509.alice2024.subsystems.drive.SwerveDrive;
import com.redstorm509.alice2024.subsystems.vision.Limelight;

public class AutoAlign extends Command {

	private SwerveDrive swerve;
	private Limelight limelight;
	private DoubleSupplier xSupplier;
	private DoubleSupplier ySupplier;
	private DoubleSupplier rotationSupplier;

	private int targetTagID;
	private boolean specificTag;

	private Translation2d outputTranslation;

	// Meant to be an "isDownBind" command
	public AutoAlign(
			SwerveDrive swerve,
			Limelight limelight,
			DoubleSupplier xSupplier,
			DoubleSupplier ySupplier,
			DoubleSupplier rotationSupplier) {
		this.swerve = swerve;
		this.limelight = limelight;
		this.xSupplier = xSupplier;
		this.ySupplier = ySupplier;
		this.rotationSupplier = rotationSupplier;

		addRequirements(swerve);
	}

	public AutoAlign(
			int alignmentTagID,
			SwerveDrive swerve,
			Limelight limelight,
			DoubleSupplier xSupplier,
			DoubleSupplier ySupplier,
			DoubleSupplier rotationSupplier) {
		this.swerve = swerve;
		this.limelight = limelight;
		this.xSupplier = xSupplier;
		this.ySupplier = ySupplier;
		this.rotationSupplier = rotationSupplier;

		this.targetTagID = alignmentTagID;

		addRequirements(swerve);
	}

	public Pose2d getAlignmentOffset(int TagID) {
		// collects desired offset position

		// CONFIGURE CUSTOM OFFSETS
		switch (TagID) {
			// AMP TAG OFFSET
			case 5: // Red Alliance
			case 6: // Blue Alliance
				return new Pose2d(new Translation2d(0, 0), new Rotation2d());

			// SPEAKER TAG OFFSET
			case 4: // Red Alliance
			case 7: // Blue Alliance
				return new Pose2d(new Translation2d(0, 0), new Rotation2d());

			// SPEAKER SIDE TAG OFFSETS
			case 3: // Red Alliance
			case 8: // Blue Alliance
				return new Pose2d(new Translation2d(0, 0), new Rotation2d());

			// STAGE TAG OFFSETS
			case 11: // Red Alliance
			case 12:
			case 13:
			case 16: // Blue Alliance
			case 15:
			case 14:
				return new Pose2d(new Translation2d(0, 0), new Rotation2d());
			default:
				return new Pose2d();
		}
	}

	@Override
	public void initialize() {
		limelight.setPipelineIndex(Constants.Vision.Pipeline.AprilTags);

		limelight.setLEDMode_ForceBlink();
	}

	@Override
	public void execute() {
		if (limelight.getTV()) {
			limelight.setLEDMode_ForceOn();
			if (!specificTag) {
				targetTagID = (int) limelight.getFiducialID();
			}
		} else {
			limelight.setLEDMode_ForceBlink();
		}

		// TEST VALUES TO MAKE SURE WORKS AS EXPECTED
		Pose3d TagPose = Limelight.toPose3D(limelight.getTargetPose_RobotSpace());
		Pose2d offsetPose = getAlignmentOffset(targetTagID);

		outputTranslation = TagPose.getTranslation().toTranslation2d().minus(offsetPose.getTranslation());

		swerve.setTargetHeading(offsetPose.getRotation().getRadians());
		if (!offsetPose.equals(new Pose2d()) || (specificTag && limelight.getFiducialID() == targetTagID)) {
			swerve.drive(
					outputTranslation,
					0,
					false, // check for issues with rotation & field relative
					false);
		} else {
			swerve.drive(
					new Translation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble()).times(Constants.kMaxSpeed),
					rotationSupplier.getAsDouble(),
					true,
					false);
		}

		SmartDashboard.putNumber("X to target position", outputTranslation.getX());
		SmartDashboard.putNumber("Y to target position", outputTranslation.getY());
		SmartDashboard.putNumber("Rotation", swerve.getYaw().getDegrees());

		SmartDashboard.putNumber("Targeted April Tag", limelight.getFiducialID());
	}

	@Override
	public boolean isFinished() {
		return MathUtil.isNear(0, outputTranslation.getX(), Constants.Vision.kAlignmentTranslationTolerance) &&
				MathUtil.isNear(0, outputTranslation.getY(), Constants.Vision.kAlignmentTranslationTolerance) &&
				MathUtil.isNear(getAlignmentOffset(targetTagID).getRotation().getDegrees(),
						swerve.getYaw().getDegrees(), Constants.Vision.kAlignmentRotationTolerance);
	}

	@Override
	public void end(boolean wasInterrupted) {
		limelight.setLEDMode_ForceOff();
		swerve.drive(new Translation2d(0, 0), 0, true, false);
	}

}
