package com.redstorm509.alice2024.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.redstorm509.alice2024.Constants;
import com.redstorm509.alice2024.subsystems.Arm;
import com.redstorm509.alice2024.subsystems.drive.SwerveDrive;
import com.redstorm509.alice2024.subsystems.vision.Limelight;
import com.redstorm509.alice2024.util.drivers.REVBlinkin;
import com.redstorm509.alice2024.util.drivers.REVBlinkin.ColorCode;

public class AutoAlign extends Command {

	private SwerveDrive swerve;
	private Arm arm;
	private Limelight limelight;
	private REVBlinkin lights;

	private DoubleSupplier xSupplier;
	private DoubleSupplier ySupplier;
	private DoubleSupplier rotationSupplier;

	private int targetTagID;
	private boolean specificTag;

	private Translation3d RobotToTag;
	private Translation2d outputTranslation;
	private double desiredArmPivot = Constants.Arm.kMinPivot;
	private double kPivotSlope = -0.710432; // TUNE ME
	private double kPivotIntercept = -27.0751; // TUNE ME
	private double kRotationSlope = 0.0; // TUNE ME
	private double kRotationIntercept = 0.0; // TUNE ME

	// Meant to be an "isDownBind" command
	public AutoAlign(
			SwerveDrive swerve,
			Arm arm,
			Limelight limelight,
			REVBlinkin lights,
			DoubleSupplier xSupplier,
			DoubleSupplier ySupplier,
			DoubleSupplier rotationSupplier) {
		this.swerve = swerve;
		this.arm = arm;
		this.limelight = limelight;
		this.lights = lights;

		this.xSupplier = xSupplier;
		this.ySupplier = ySupplier;
		this.rotationSupplier = rotationSupplier;

		this.specificTag = false;

		addRequirements(swerve, arm, lights);
	}

	public Pose2d getAlignmentOffset(int TagID) {
		// collects desired offset position
		// Translation part of pose should be the desired translational offset.
		// Rotation part of pose should be desired angle heading.

		double desiredRotation;

		// CONFIGURE CUSTOM OFFSETS
		switch (TagID) {
			// AMP TAG OFFSET
			case 5: // Red Alliance
			case 6: // Blue Alliance
				// desiredRotation = Math.toRadians(limelight.getTX())
				// - (Math.atan(Math.abs(RobotToTag.getX()) / Math.abs(RobotToTag.getY())));
				return new Pose2d(new Translation2d(0, 0), new Rotation2d());

			// SPEAKER TAG OFFSET
			case 4: // Red Alliance
			case 7: // Blue Alliance
				/*-
				desiredRotation = Math.toRadians(
						-limelight.getTX() + (kRotationSlope * swerve.getYaw().getDegrees() + kRotationIntercept))
						* 4.5;
				SmartDashboard.putNumber("Desired Rotation",
						(kRotationSlope * swerve.getYaw().getDegrees() + kRotationIntercept));*/

				desiredRotation = Math.toRadians(-limelight.getTX());
				desiredArmPivot = kPivotSlope * limelight.getTY() + kPivotIntercept;
				if (!limelight.getTV()) {
					desiredArmPivot = arm.getPivotDegrees();
				}

				return new Pose2d(new Translation2d(0, 0), new Rotation2d(desiredRotation));

			// SPEAKER SIDE TAG OFFSETS (43 cm to the right of central tags)
			case 3: // Red Alliance
			case 8: // Blue Alliance
				// skip and wait to rotate to center tag
				return new Pose2d();

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

		lights.setColor(ColorCode.AutoTargetLost);

		SmartDashboard.putBoolean("Autonomous Lock On", false);

		Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
		if (alliance.isPresent()) {
			if (alliance.get() == DriverStation.Alliance.Blue) {
				limelight.setPriorityTagID(7);
			} else {
				limelight.setPriorityTagID(4);
			}
		}
	}

	@Override
	public void execute() {
		if (limelight.getTV()) {
			if ((int) limelight.getFiducialID() == 8 || (int) limelight.getFiducialID() == 3) {
				lights.setColor(ColorCode.AutoTargetLost);
				SmartDashboard.putBoolean("Autonomous Lock On", false);
			} else {
				lights.setColor(ColorCode.AutoTargetFound);
				SmartDashboard.putBoolean("Autonomous Lock On", true);
			}
			if (!specificTag) {
				targetTagID = (int) limelight.getFiducialID();
			}
		} else {
			lights.setColor(ColorCode.AutoTargetLost);
			SmartDashboard.putBoolean("Autonomous Lock On", false);
		}

		// TEST VALUES TO MAKE SURE WORKS AS EXPECTED

		Pose3d TagToRobotPose = Limelight.toPose3D(limelight.getBotPose_TargetSpace());
		RobotToTag = new Translation3d(-TagToRobotPose.getZ(), -TagToRobotPose.getX(), -TagToRobotPose.getY());

		Pose2d offsetPose = getAlignmentOffset(targetTagID);
		outputTranslation = RobotToTag.toTranslation2d().minus(offsetPose.getTranslation());

		// offsetPose = new Pose2d();
		// checks if has valid tag, and if it is specific tag when aplicable
		if ((!specificTag && !offsetPose.equals(new Pose2d()))
				|| (specificTag && limelight.getFiducialID() == targetTagID)) {
			if (!offsetPose.getTranslation().equals(new Translation2d())) {
				swerve.setTargetHeading(offsetPose.getRotation().getDegrees());

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
						MathUtil.clamp(offsetPose.getRotation().getRadians(), -Constants.kMaxAngularVelocity,
								Constants.kMaxAngularVelocity),
						true,
						true);
			}

			// SmartDashboard.putNumber("Desired Arm Pivot", desiredArmPivot);

			arm.setPivotDegrees(MathUtil.clamp(desiredArmPivot, Constants.Arm.kMinPivot, Constants.Arm.kMaxPivot));
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

		// SmartDashboard.putNumber("TY", limelight.getTY());

		// SmartDashboard.putNumber("X to Tag", RobotToTag.getX());
		// SmartDashboard.putNumber("Y to Tag", RobotToTag.getY());
		// SmartDashboard.putNumber("Z to tag", RobotToTag.getZ());

		/*-
		SmartDashboard.putNumber("TX", limelight.getTX());
		SmartDashboard.putNumber("tan angle", Math.atan(Math.abs(RobotToTag.getX()) / Math.abs(RobotToTag.getY())));
		SmartDashboard.putNumber("Output Rotation",
				-limelight.getTX()
						- Math.toDegrees(Math.atan(Math.abs(RobotToTag.getX()) / Math.abs(RobotToTag.getY()))));
		*/
		// SmartDashboard.putNumber("X to target position", outputTranslation.getX());
		// SmartDashboard.putNumber("Y to target position", outputTranslation.getY());
	}

	@Override
	public boolean isFinished() {
		/*-
		return MathUtil.isNear(0, outputTranslation.getX(), Constants.Vision.kAlignmentTranslationTolerance) &&
				MathUtil.isNear(0, outputTranslation.getY(), Constants.Vision.kAlignmentTranslationTolerance) &&
				MathUtil.isNear(getAlignmentOffset(targetTagID).getRotation().getDegrees(),
						swerve.getYaw().getDegrees(), Constants.Vision.kAlignmentRotationTolerance);
		 */
		return false;
	}

	@Override
	public void end(boolean wasInterrupted) {
		lights.setDefault();
		SmartDashboard.putBoolean("Autonomous Lock On", false);
		swerve.drive(new Translation2d(0, 0), 0, true, false);
		swerve.setTargetHeading(swerve.getYaw().getDegrees());
	}
}
