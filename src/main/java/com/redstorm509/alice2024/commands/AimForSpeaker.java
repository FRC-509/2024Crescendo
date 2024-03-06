package com.redstorm509.alice2024.commands;

import java.util.function.DoubleSupplier;

import com.redstorm509.alice2024.Constants;
import com.redstorm509.alice2024.subsystems.Arm;
import com.redstorm509.alice2024.subsystems.drive.SwerveDrive;
import com.redstorm509.alice2024.subsystems.vision.Limelight;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AimForSpeaker extends Command {

	private SwerveDrive swerve;
	private Arm arm;
	private DoubleSupplier translationXSupplier;
	private DoubleSupplier translationYSupplier;
	private DoubleSupplier rotationSupplier;
	private Limelight limelight;
	private boolean shouldAbort;

	@Deprecated
	public AimForSpeaker(
			SwerveDrive swerve,
			Arm arm,
			DoubleSupplier translationXSupplier,
			DoubleSupplier translationYSupplier,
			DoubleSupplier rotationSupplier,
			Limelight limelight) {
		this.swerve = swerve;
		this.arm = arm;
		this.translationXSupplier = translationXSupplier;
		this.translationYSupplier = translationYSupplier;
		this.rotationSupplier = rotationSupplier;
		this.limelight = limelight;

		addRequirements(swerve, arm);
	}

	@Override
	public void initialize() {
		limelight.setPipelineIndex(Constants.Vision.Pipeline.AprilTags);

		limelight.setLEDMode_ForceBlink();

		shouldAbort = !limelight.getTV();
	}

	@Override
	public void execute() {
		double rot = rotationSupplier.getAsDouble();
		if (limelight.getTV()) {
			limelight.setLEDMode_ForceOn();
			Pose3d pose = Limelight.toPose3D(limelight.getTargetPose_RobotSpace());
			double distance = Math.hypot(pose.getX(), pose.getZ()) + Units.inchesToMeters(10);
			double height = Math.abs(pose.getY()) + Units.inchesToMeters(30);
			double targetAngle = Math.toDegrees(Math.atan(2 * height / distance))
					- Constants.Shooter.kPivotToShootAngleOffset;

			// setPivotDegrees() not working so really bad solution for now (fightin issues)
			arm.setPivotDegrees(MathUtil.clamp(targetAngle, 0.0, Constants.Shooter.kMaxPivot));

			SmartDashboard.putNumber("Pivot Target Angle", targetAngle);
		} else {
			limelight.setLEDMode_ForceBlink();
		}
		swerve.drive(
				new Translation2d(translationXSupplier.getAsDouble(), translationYSupplier.getAsDouble())

						.times(Constants.kMaxSpeed),
				rot,
				true,
				false);
		SmartDashboard.putNumber("TX FOR ARM CAMERA", limelight.getTX());
	}

	@Override
	public boolean isFinished() {
		return shouldAbort;
	}

	@Override
	public void end(boolean wasInterrupted) {
		limelight.setLEDMode_ForceOff();
		swerve.drive(new Translation2d(0, 0), 0, true, false);
	}
}
