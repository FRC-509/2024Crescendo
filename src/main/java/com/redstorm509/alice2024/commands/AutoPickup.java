package com.redstorm509.alice2024.commands;

import com.redstorm509.alice2024.Constants;
import com.redstorm509.alice2024.subsystems.Intake;
import com.redstorm509.alice2024.subsystems.drive.SwerveDrive;
import com.redstorm509.alice2024.subsystems.vision.Limelight;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoPickup extends Command {

	private SwerveDrive swerve;
	private Limelight limelight;
	private Intake intake;
	private boolean beganIntaking;
	private boolean shouldAbort;

	@Deprecated
	public AutoPickup(SwerveDrive swerve, Limelight limelight, Intake intake) {
		this.swerve = swerve;
		this.limelight = limelight;
		this.intake = intake;

		addRequirements(swerve, intake);
	}

	@Override
	public void initialize() {
		shouldAbort = !limelight.getTV();

		beganIntaking = false;

		limelight.setLEDMode_ForceOn();

		this.limelight.setPipelineIndex(Constants.Vision.Pipeline.NeuralNetwork);

		// set setpoint for pid
	}

	@Override
	public void execute() {
		if (!limelight.getTV()) {
			return;
		}

		// Math to get distance from target
		double angleToTarget = -limelight.getTY() + -Constants.Vision.kIntakeCameraAngleOffset;
		double distanceToTarget = Constants.Vision.kIntakeCameraHeightFromGround
				/ Math.tan(Math.toRadians(angleToTarget));
		double outputMove = -distanceToTarget * 2;
		if (outputMove > Constants.kMaxSpeed) {
			outputMove = Constants.kMaxSpeed;
		}
		swerve.drive(new Translation2d(0.0,
				outputMove),
				Math.toRadians(limelight.getTX() * 3),
				false, false);
		if (distanceToTarget < 2 || beganIntaking) {
			beganIntaking = true;
			intake.intake(true);
		}
		SmartDashboard.putNumber("Angle To Target", angleToTarget);
		SmartDashboard.putNumber("Distance From Target", distanceToTarget);
	}

	@Override
	public boolean isFinished() {
		return shouldAbort;
	}

	@Override
	public void end(boolean wasInterrupted) {
		swerve.drive(new Translation2d(0, 0), 0, true, false);
		intake.stop();
	}
}