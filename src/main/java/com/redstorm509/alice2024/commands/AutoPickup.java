package com.redstorm509.alice2024.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.redstorm509.alice2024.Constants;
import com.redstorm509.alice2024.subsystems.drive.SwerveDrive;
import com.redstorm509.alice2024.subsystems.vision.Limelight;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoPickup extends Command {

	private SwerveDrive swerve;
	private Limelight limelight;
	private BooleanSupplier run;

	public AutoPickup(SwerveDrive swerve, Limelight limelight, BooleanSupplier run) {
		// this.swerve = swerve;
		this.limelight = limelight;
		this.run = run;

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

		// SmartDashboard.putNumber("limelight offset: ", limelight.getXOffset());
		// strafePID.debug("strafe");

		// Strafe using a PID on the limelight's X offset to bring it to zero.
		Translation2d trans = Limelight.toPose3D(limelight.getTargetPose_RobotSpace()).getTranslation()
				.toTranslation2d().times(0.4 * Constants.kMaxSpeed);
		swerve.drive(trans,
				Math.toRadians(limelight.getTX()),
				true, false);
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