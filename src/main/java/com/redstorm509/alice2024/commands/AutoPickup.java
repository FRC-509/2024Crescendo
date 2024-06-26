package com.redstorm509.alice2024.commands;

import java.util.function.DoubleSupplier;

import com.redstorm509.alice2024.Constants;
import com.redstorm509.alice2024.subsystems.Indexer;
import com.redstorm509.alice2024.subsystems.Intake;
import com.redstorm509.alice2024.subsystems.Indexer.IndexerState;
import com.redstorm509.alice2024.subsystems.drive.SwerveDrive;
import com.redstorm509.alice2024.subsystems.vision.Limelight;
import com.redstorm509.alice2024.util.drivers.REVBlinkin;
import com.redstorm509.alice2024.util.drivers.REVBlinkin.ColorCode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoPickup extends Command {

	private SwerveDrive swerve;
	private Limelight limelight;
	private Intake intake;
	private Indexer indexer;
	private boolean beganIntaking;
	private DoubleSupplier xSupplier;
	private DoubleSupplier ySupplier;
	private DoubleSupplier rotationSupplier;
	private REVBlinkin lights;

	private boolean isFinished = false;

	private double lastTX;
	private double lastDistanceToTarget;
	private boolean lostTarget = false;

	public AutoPickup(
			SwerveDrive swerve,
			Limelight limelight,
			Intake intake,
			Indexer indexer,
			REVBlinkin lights,
			DoubleSupplier xSupplier,
			DoubleSupplier ySupplier,
			DoubleSupplier rotationSupplier) {
		this.swerve = swerve;
		this.limelight = limelight;
		this.intake = intake;
		this.indexer = indexer;
		this.lights = lights;

		this.xSupplier = xSupplier;
		this.ySupplier = ySupplier;
		this.rotationSupplier = rotationSupplier;

		addRequirements(swerve, intake, indexer, lights);
	}

	@Override
	public void initialize() {
		isFinished = false;
		beganIntaking = false;
		lostTarget = false;
		lastTX = 0.0;
		lastDistanceToTarget = (Constants.Vision.kIntakeCameraHeightFromGround
				/ Math.tan(Math.toRadians(-limelight.getTY() + -Constants.Vision.kIntakeCameraAngleOffset)));

		lights.setColor(ColorCode.AutoTargetLost);
		SmartDashboard.putBoolean("Autonomous Lock On", false);
		limelight.setPipelineIndex(Constants.Vision.Pipeline.NeuralNetwork);
	}

	@Override
	public void execute() {
		if (!limelight.getTV() && !beganIntaking) {
			lights.setColor(ColorCode.AutoTargetLost);
			SmartDashboard.putBoolean("Autonomous Lock On", false);
		} else {
			lights.setColor(ColorCode.AutoTargetFound);
			SmartDashboard.putBoolean("Autonomous Lock On", true);
		}

		// Finds distance to target and how much to move
		double angleToTarget = -limelight.getTY() + -Constants.Vision.kIntakeCameraAngleOffset;
		double distanceToTargetY = (Constants.Vision.kIntakeCameraHeightFromGround
				/ Math.tan(Math.toRadians(angleToTarget)));
		double distanceToTargetX = -(distanceToTargetY * Math.sin(Math.toRadians(limelight.getTX())));

		if (indexer.indexingNoteState != IndexerState.Noteless || !limelight.getTV()) {
			swerve.drive(
					new Translation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble()).times(Constants.kMaxSpeed),
					rotationSupplier.getAsDouble() * Constants.kMaxAngularVelocity,
					true,
					false);
		} else if ((limelight.getTV() && indexer.indexingNoteState == IndexerState.Noteless
				&& distanceToTargetY < 3.0)) {
			double travelDistanceY;
			double useTX;
			if (!lostTarget) {
				if (Math.abs(distanceToTargetY) < Math
						.abs(lastDistanceToTarget + Constants.Vision.kMaxTargetDistanceVariation)) {
					travelDistanceY = distanceToTargetY;
					useTX = limelight.getTX();
				} else {
					travelDistanceY = lastDistanceToTarget;
					useTX = lastTX;
					lostTarget = true;
				}
			} else {
				travelDistanceY = lastDistanceToTarget / 2;
				useTX = lastTX / 2;
				lights.setColor(ColorCode.AutoTargetLost);
				SmartDashboard.putBoolean("Autonomous Lock On", false);
			}

			swerve.drive(
					new Translation2d(
							MathUtil.clamp(travelDistanceY * 2, -Constants.kMaxSpeed, Constants.kMaxSpeed),
							MathUtil.clamp(distanceToTargetX * 3, -Constants.kMaxSpeed, Constants.kMaxSpeed)),
					MathUtil.clamp(Math.toRadians(-useTX * 3),
							-Constants.kMaxAngularVelocity, Constants.kMaxAngularVelocity),
					false,
					false);
		} else {
			swerve.drive(
					new Translation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble()).times(Constants.kMaxSpeed),
					indexer.indexingNoteState == IndexerState.Noteless ? MathUtil.clamp(Math.toRadians(lastTX * 1.5),
							-Constants.kMaxAngularVelocity, Constants.kMaxAngularVelocity)
							: rotationSupplier.getAsDouble() * Constants.kMaxAngularVelocity,
					true,
					false);
		}

		if (distanceToTargetY < 2 || beganIntaking) {
			beganIntaking = true;

			// has note logic using beam breaks
			if (indexer.indexingNoteState == IndexerState.HasNote) {
				isFinished = true;
			} else if (indexer.indexingNoteState == IndexerState.Noteless) {
				indexer.rawIndexer(-Constants.Indexer.kSpinSpeed);
				intake.intake(true);
			} else if (indexer.indexingNoteState == IndexerState.NoteTooShooter) {
				indexer.rawIndexer(Constants.Indexer.kReducedSpinSpeed);
				intake.stop();
			} else if (indexer.indexingNoteState == IndexerState.NoteTooShooterExtreme) {
				indexer.rawIndexer(Constants.Indexer.kSpinSpeed);
				intake.stop();
			} else if (indexer.indexingNoteState == IndexerState.NoteTooIntake) {
				indexer.rawIndexer(-Constants.Indexer.kReducedSpinSpeed);
				intake.stop();
			} else if (indexer.indexingNoteState == IndexerState.NoteTooIntakeExtreme) {
				indexer.rawIndexer(-Constants.Indexer.kSpinSpeed);
				intake.intake(true);
			}
		}
	}

	@Override
	public boolean isFinished() {
		return isFinished;
	}

	@Override
	public void end(boolean wasInterrupted) {
		swerve.drive(new Translation2d(0, 0), 0, true, false);
		if (indexer.indexingNoteState == IndexerState.HasNote) {
			lights.setColor(ColorCode.HasNote);
		} else {
			lights.setDefault();
		}
		SmartDashboard.putBoolean("Autonomous Lock On", false);
		intake.stop();
		indexer.rawIndexer(0.0);
	}
}