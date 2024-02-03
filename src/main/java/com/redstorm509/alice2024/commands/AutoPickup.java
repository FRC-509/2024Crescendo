package com.redstorm509.alice2024.commands;

import java.util.function.BooleanSupplier;
import com.redstorm509.alice2024.Constants;
import com.redstorm509.alice2024.subsystems.drive.SwerveDrive;
import com.redstorm509.alice2024.subsystems.vision.Limelight;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoPickup extends Command {

	private SwerveDrive swerve;
	private Limelight limelight;
	private BooleanSupplier run;

	public AutoPickup(SwerveDrive swerve, Limelight limelight) {
		// this.swerve = swerve;
		this.limelight = limelight;

		addRequirements(swerve);
	}

	@Override
	public void initialize() {
		if (!limelight.getTV()) {
			end(true);
		}

		this.limelight.setPipelineIndex(Constants.Vision.NeuralNetworkPipeline);
		this.limelight.setLEDMode_ForceOff();

		// set setpoint for pid
	}

	@Override
	public void execute() {
		if (!limelight.getTV()) {
			return;
		}

		// Math to get distance from target
		double angleToTarget = Math.abs(limelight.getTY()) + Constants.Vision.intakeCAmeraAngleOffset;
		swerve.drive(new Translation2d(0.0, Constants.Vision.intakeCameraHeightFromGround / Math.tan(angleToTarget)),
				Math.toRadians(limelight.getTX()),
				false, false);

		SmartDashboard.putNumber("Angle To Target", angleToTarget);
		SmartDashboard.putNumber("Distance From Target",
				Constants.Vision.intakeCameraHeightFromGround / Math.tan(angleToTarget));
	}

	@Override
	public boolean isFinished() {
		return !run.getAsBoolean();
	}

	@Override
	public void end(boolean wasInterrupted) {
		swerve.drive(new Translation2d(0, 0), 0, true, false);
	}
}