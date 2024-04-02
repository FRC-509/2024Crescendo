package com.redstorm509.alice2024.commands.autonomous;

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

public class AutonomousAutoPickup extends Command {

	private SwerveDrive swerve;
	private Limelight limelight;
	private Intake intake;
	private Indexer indexer;
	private REVBlinkin lights;

	private boolean beganIntaking;

	private boolean isFinished = false;

	private double lastTX;
	private double lastDistanceToTarget;
	private boolean lostTarget = false;

	public AutonomousAutoPickup(
			SwerveDrive swerve,
			Limelight limelight,
			Intake intake,
			Indexer indexer,
			REVBlinkin lights) {
		this.swerve = swerve;
		this.limelight = limelight;
		this.intake = intake;
		this.indexer = indexer;
		this.lights = lights;

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

		// limelight.setLEDMode_ForceBlink();
		lights.setColor(ColorCode.AutoTargetLost);
		SmartDashboard.putBoolean("Autonomous Lock On", false);
		limelight.setPipelineIndex(Constants.Vision.Pipeline.NeuralNetwork);
	}

	@Override
	public void execute() {
		if (!limelight.getTV() && !beganIntaking) {
			// limelight.setLEDMode_ForceBlink();
			lights.setColor(ColorCode.AutoTargetLost);
			SmartDashboard.putBoolean("Autonomous Lock On", false);
		} else {
			// limelight.setLEDMode_ForceOn();
			lights.setColor(ColorCode.AutoTargetFound);
			SmartDashboard.putBoolean("Autonomous Lock On", true);
		}

		// Finds distance to target and how much to move
		double angleToTarget = -limelight.getTY() + -Constants.Vision.kIntakeCameraAngleOffset;
		double distanceToTargetY = (Constants.Vision.kIntakeCameraHeightFromGround
				/ Math.tan(Math.toRadians(angleToTarget)));
		double distanceToTargetX = -(distanceToTargetY * Math.sin(Math.toRadians(limelight.getTX())));

		// double check correct sign, double negatives l

		if ((limelight.getTV() && indexer.indexingNoteState == IndexerState.Noteless
				&& distanceToTargetY < 3.0)) {
			// checks if target is the same target that has been tracking, if not follows
			// last known path (slightly jank, but test)
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
				// limelight.setLEDMode_ForceBlink();
				lights.setColor(ColorCode.AutoTargetLost);
				SmartDashboard.putBoolean("Autonomous Lock On", false);
			}

			swerve.drive(
					new Translation2d(
							MathUtil.clamp(travelDistanceY * 2, -Constants.kMaxSpeed, Constants.kMaxSpeed), // tune
							MathUtil.clamp(distanceToTargetX * 3, -Constants.kMaxSpeed, Constants.kMaxSpeed)),
					MathUtil.clamp(Math.toRadians(-useTX * 3),
							-Constants.kMaxAngularVelocity, Constants.kMaxAngularVelocity), // tune this
					false,
					false);
		} else {
			double spinDirection = swerve.getYaw().getDegrees() > 0.0 ? 1.0 : -1.0; // REPLACE WITH WHAT FORWARD IS
			swerve.drive(
					new Translation2d(),
					Math.toRadians(5.0 * spinDirection), // just spin lol
					true,
					false);
		}

		if (distanceToTargetY < 2 || beganIntaking) {
			beganIntaking = true;

			isFinished = indexer.indexingNoteState != IndexerState.Noteless;
			indexer.rawIndexer(-Constants.Indexer.kSpinSpeed);
			intake.intake(true);
		}
	}

	@Override
	public boolean isFinished() {
		return isFinished;
	}

	@Override
	public void end(boolean wasInterrupted) {
		swerve.drive(new Translation2d(0, 0), 0, true, false);
		// limelight.setLEDMode_ForceOff();
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