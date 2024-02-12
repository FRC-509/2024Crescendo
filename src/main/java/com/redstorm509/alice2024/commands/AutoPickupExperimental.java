package com.redstorm509.alice2024.commands;

import java.util.function.DoubleSupplier;
import com.redstorm509.alice2024.Constants;
import com.redstorm509.alice2024.subsystems.Intake;
import com.redstorm509.alice2024.subsystems.drive.SwerveDrive;
import com.redstorm509.alice2024.subsystems.vision.Limelight;
import com.redstorm509.alice2024.util.devices.VL53L4CD;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoPickupExperimental extends Command {

	private SwerveDrive swerve;
	private Limelight limelight;
	private Intake intake;
	private boolean beganIntaking;
	private DoubleSupplier xSupplier;
	private DoubleSupplier ySupplier;
	private double stickMagnitude;
	private double startingMagnitude;
	private boolean usesMagnitudeCondition;

	public AutoPickupExperimental(
			SwerveDrive swerve,
			Limelight limelight,
			Intake intake,
			DoubleSupplier xSupplier,
			DoubleSupplier ySupplier) {
		this.swerve = swerve;
		this.limelight = limelight;
		this.intake = intake;

		this.xSupplier = xSupplier;
		this.ySupplier = ySupplier;

		usesMagnitudeCondition = true;

		addRequirements(swerve, intake);
	}

	public AutoPickupExperimental(
			SwerveDrive swerve,
			Limelight limelight,
			Intake intake) {
		this.swerve = swerve;
		this.limelight = limelight;
		this.intake = intake;

		usesMagnitudeCondition = false;

		addRequirements(swerve);
	}

	@Override
	public void initialize() {
		if (!limelight.getTV()) {
			end(true);
		}
		beganIntaking = false;

		// sets the starting magnitude to the
		if (usesMagnitudeCondition) {
			if (Math.abs(ySupplier.getAsDouble()) >= Math.abs(xSupplier.getAsDouble())) {
				startingMagnitude = Math.abs(ySupplier.getAsDouble());
			} else {
				startingMagnitude = Math.abs(xSupplier.getAsDouble());
			}
		}

		limelight.setLEDMode_ForceOn();

		limelight.setPipelineIndex(Constants.Vision.Pipeline.NeuralNetwork);
	}

	@Override
	public void execute() {
		// Checks if limelight has a target
		if (!limelight.getTV()) {
			limelight.setLEDMode_ForceBlink();
			return;
		} else {
			limelight.setLEDMode_ForceOn();
		}

		// sets the Stick magnitude to the highest value
		if (usesMagnitudeCondition) {
			if (Math.abs(ySupplier.getAsDouble()) >= Math.abs(xSupplier.getAsDouble())) {
				stickMagnitude = Math.abs(ySupplier.getAsDouble());
			} else {
				stickMagnitude = Math.abs(xSupplier.getAsDouble());
			}
		}

		// Finds distance to target and how much to move
		double angleToTarget = -limelight.getTY() + -Constants.Vision.kIntakeCameraAngleOffset;
		double distanceToTarget = Constants.Vision.kIntakeCameraHeightFromGround
				/ Math.tan(Math.toRadians(angleToTarget));
		double outputMove = -distanceToTarget * 2; // make sure distanceToTarget has correct sign, fix double negatives
		if (outputMove > Constants.kMaxSpeed) {
			outputMove = Constants.kMaxSpeed;
		}

		swerve.drive(new Translation2d(0.0, // possibly change 0.0 to: -distanceToTarget * getTX() or scale somehow
				outputMove),
				Math.toRadians(distanceToTarget * limelight.getTX() * 1.5), // tune this
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
		if (usesMagnitudeCondition) {
			// ends if the stick crosses the center, ends command
			return stickMagnitude < startingMagnitude / 2.5 || stickMagnitude < 0.1;
		}
		return false; // add an || for if note sensors detect note in pipeline
	}

	@Override
	public void end(boolean wasInterrupted) {
		swerve.drive(new Translation2d(0, 0), 0, true, false);
		limelight.setLEDMode_ForceOff();
		intake.stop();
	}
}