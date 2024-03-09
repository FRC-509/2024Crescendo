package com.redstorm509.alice2024.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;

import com.redstorm509.alice2024.Constants;
import com.redstorm509.alice2024.subsystems.Arm;
import com.redstorm509.alice2024.subsystems.drive.SwerveDrive;
import com.redstorm509.alice2024.subsystems.vision.Limelight;

public class AutoAlign extends Command {

	private SwerveDrive swerve;
	private Arm arm;
	private Limelight limelight;
	private DoubleSupplier xSupplier;
	private DoubleSupplier ySupplier;
	private DoubleSupplier rotationSupplier;

	private int targetTagID;
	private boolean specificTag;

	private Translation3d RobotToTag;
	private Translation2d outputTranslation;
	private double desiredArmPivot;

	// Meant to be an "isDownBind" command
	public AutoAlign(
			SwerveDrive swerve,
			Arm arm,
			Limelight limelight,
			DoubleSupplier xSupplier,
			DoubleSupplier ySupplier,
			DoubleSupplier rotationSupplier) {
		this.swerve = swerve;
		this.arm = arm;
		this.limelight = limelight;
		this.xSupplier = xSupplier;
		this.ySupplier = ySupplier;
		this.rotationSupplier = rotationSupplier;

		addRequirements(swerve, arm);
	}

	public AutoAlign(
			int alignmentTagID,
			SwerveDrive swerve,
			Arm arm,
			Limelight limelight,
			DoubleSupplier xSupplier,
			DoubleSupplier ySupplier,
			DoubleSupplier rotationSupplier) {
		this.swerve = swerve;
		this.arm = arm;
		this.limelight = limelight;
		this.xSupplier = xSupplier;
		this.ySupplier = ySupplier;
		this.rotationSupplier = rotationSupplier;

		this.targetTagID = alignmentTagID;

		addRequirements(swerve, arm);
	}

	public Pose2d getAlignmentOffset(int TagID) {
		// collects desired offset position
		// Translation part of pose should be offset wanted offset
		// Rotation part of pose should be desired angle heading

		double desiredRotation;

		// CONFIGURE CUSTOM OFFSETS
		switch (TagID) {
			// AMP TAG OFFSET
			case 5: // Red Alliance
			case 6: // Blue Alliance
				return new Pose2d(new Translation2d(0, 0), new Rotation2d());

			// SPEAKER TAG OFFSET
			case 4: // Red Alliance
			case 7: // Blue Alliance
				// robot shoots from behind, so opposite of this angle as heading (?)
				desiredRotation = -Math.atan(RobotToTag.getX() / RobotToTag.getY());

				// VERIFY SIGNS & AXES
				desiredArmPivot = Math.toDegrees(Math.atan(
						2 * RobotToTag.getZ() / Math.hypot(RobotToTag.getX(), RobotToTag.getY())))
						- Constants.Arm.kPivotToShootAngleOffset;

				return new Pose2d(new Translation2d(0, 0),
						new Rotation2d(swerve.getYaw().getRadians() + Math.toRadians(desiredRotation)));

			// SPEAKER SIDE TAG OFFSETS (43 cm to the right of central tags)
			case 3: // Red Alliance
			case 8: // Blue Alliance
				desiredRotation = -Math.atan((RobotToTag.getX() - 0.43) / RobotToTag.getY());

				// VERIFY SIGNS & AXES
				desiredArmPivot = Math.toDegrees(Math.atan(
						2 * RobotToTag.getZ() / Math.hypot(RobotToTag.getX() - 0.43, RobotToTag.getY())))
						- Constants.Arm.kPivotToShootAngleOffset;

				return new Pose2d(new Translation2d(), new Rotation2d(desiredRotation));

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

		// inverted tag centric version
		Pose3d TagToRobotPose = Limelight.toPose3D(limelight.getTargetPose_RobotSpace());
		RobotToTag = new Translation3d(TagToRobotPose.getX(), -TagToRobotPose.getZ(), -TagToRobotPose.getY());

		Pose2d offsetPose = getAlignmentOffset(targetTagID);
		outputTranslation = RobotToTag.toTranslation2d().minus(offsetPose.getTranslation());

		// checks if has valid tag, and if it is specific tag when aplicable
		if ((!specificTag && !offsetPose.equals(new Pose2d()))
				|| (specificTag && limelight.getFiducialID() == targetTagID)) {
			swerve.setTargetHeading(offsetPose.getRotation().getDegrees() + 90);
			if (!offsetPose.getTranslation().equals(new Translation2d())) {
				// possible wait for target heading to be near reached if needed
				swerve.drive(
						new Translation2d(
								MathUtil.clamp(outputTranslation.getX(), -Constants.kMaxSpeed, Constants.kMaxSpeed),
								MathUtil.clamp(outputTranslation.getY(), -Constants.kMaxSpeed, Constants.kMaxSpeed)),
						0.0,
						false, // check for issues with rotation & field relative
						false);
			} else {
				swerve.drive(
						new Translation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble()).times(Constants.kMaxSpeed),
						0.0,
						true,
						false);
			}

			SmartDashboard.putNumber("Desired Arm Pivot",
					MathUtil.clamp(desiredArmPivot + Constants.Arm.kMinPivot, Constants.Arm.kMinPivot,
							Constants.Arm.kMaxPivot));
			arm.setPivotDegrees(
					MathUtil.clamp(desiredArmPivot + Constants.Arm.kMinPivot, Constants.Arm.kMinPivot,
							Constants.Arm.kMaxPivot));
		} else {
			// if valid tag but no translation, sets desired rotation with operator
			// movement, otherwise full operator control
			swerve.drive(
					new Translation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble()).times(Constants.kMaxSpeed),
					rotationSupplier.getAsDouble() * Constants.kMaxAngularVelocity,
					true,
					false);
		}

		SmartDashboard.putNumber("Targeted April Tag", limelight.getFiducialID());

		SmartDashboard.putNumber("X to Tag", RobotToTag.getX());
		SmartDashboard.putNumber("Y to Tag", RobotToTag.getY());

		SmartDashboard.putNumber("X to target position", outputTranslation.getX());
		SmartDashboard.putNumber("Y to target position", outputTranslation.getY());

		SmartDashboard.putNumber("Degrees to Target Heading",
				offsetPose.getRotation().getDegrees() - swerve.getYaw().getDegrees());
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
