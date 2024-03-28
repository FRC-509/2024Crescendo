package com.redstorm509.alice2024.commands.autonomous;

import com.redstorm509.alice2024.subsystems.Intake;
import com.redstorm509.alice2024.subsystems.Indexer.IndexerState;

import java.util.Optional;

import com.pathplanner.lib.commands.PathfindHolonomic;
import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.redstorm509.alice2024.Constants;
import com.redstorm509.alice2024.subsystems.Indexer;
import com.redstorm509.alice2024.subsystems.drive.SwerveDrive;
import com.redstorm509.alice2024.subsystems.vision.Limelight;
import com.redstorm509.alice2024.util.drivers.REVBlinkin;
import com.redstorm509.alice2024.util.drivers.REVBlinkin.ColorCode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AdaptableAutoPickup extends Command {

	private SwerveDrive swerve;
	private Limelight limelight;
	private Intake intake;
	private Indexer indexer;
	private boolean beganIntaking;

	private REVBlinkin lights;

	private boolean isFinished = false;

	private double lastTX;
	private double lastDistanceToTarget;
	private boolean lostTarget = false;

	private Pose2d poseToDriveBackTo = new Pose2d();

	public AdaptableAutoPickup(
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

		limelight.setLEDMode_ForceBlink();
		lights.setColor(ColorCode.AutoTargetLost);
		SmartDashboard.putBoolean("Autonomous Lock On", false);
		limelight.setPipelineIndex(Constants.Vision.Pipeline.NeuralNetwork);

		poseToDriveBackTo = swerve.getRawOdometeryPose();
	}

	@Override
	public void execute() {
		if (!limelight.getTV() && !beganIntaking) {
			// add behavior
		} else {
			limelight.setLEDMode_ForceOn();
			lights.setColor(ColorCode.AutoTargetFound);
			SmartDashboard.putBoolean("Autonomous Lock On", true);
		}

		// Finds distance to target and how much to move
		double angleToTarget = -limelight.getTY() + -Constants.Vision.kIntakeCameraAngleOffset;
		double distanceToTargetY = (Constants.Vision.kIntakeCameraHeightFromGround
				/ Math.tan(Math.toRadians(angleToTarget)));
		double distanceToTargetX = -(distanceToTargetY * Math.sin(Math.toRadians(limelight.getTX())));

		// double check correct sign, double negatives l

		if (indexer.indexingNoteState == IndexerState.HasNote) {
			// if command has grabbed note
			PathfindHolonomic pathfindercmd = new PathfindHolonomic(
					poseToDriveBackTo,
					new PathConstraints(5.02, 5.02, Math.toRadians(660), Math.toRadians(660)),
					swerve::getEstimatedPose,
					swerve::getChassisSpeeds,
					swerve::setChassisSpeeds,
					swerve.getHolonomicPathFollowerConfig(),
					swerve);
			isFinished = true;
			pathfindercmd.schedule();

		} else if ((limelight.getTV() && indexer.indexingNoteState == IndexerState.Noteless
				&& distanceToTargetY < 4.0)) {
			// checks if target is the same target that has been tracking, if not follows
			// last known path (slightly jank, but test)
			double travelDistanceY;
			double useTX;

			// Lost target logic and corrections
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
				limelight.setLEDMode_ForceBlink();
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
		}

		if (distanceToTargetY < 2 || beganIntaking) {
			beganIntaking = true;

			// has note logic using beam breaks

			// CHANGE TO CONSTANTS
			if (indexer.indexingNoteState == IndexerState.HasNote) {
				isFinished = true;

				lights.setColor(ColorCode.HasNote);
			} else if (indexer.indexingNoteState == IndexerState.Noteless) {
				indexer.rawIndexer(-Constants.Indexer.kSpinSpeed);
				intake.intake(true);

				lights.setColor(ColorCode.NoteInsideRobot);
			} else if (indexer.indexingNoteState == IndexerState.NoteTooShooter) {
				indexer.rawIndexer(Constants.Indexer.kReducedSpinSpeed); // increase if needed
				intake.stop();

				lights.setColor(ColorCode.NoteInsideRobot);
			} else if (indexer.indexingNoteState == IndexerState.NoteTooShooterExtreme) {
				indexer.rawIndexer(Constants.Indexer.kSpinSpeed);
				intake.stop();

				lights.setColor(ColorCode.NoteInsideRobot);
			} else if (indexer.indexingNoteState == IndexerState.NoteTooIntake) {
				indexer.rawIndexer(-Constants.Indexer.kReducedSpinSpeed); // increase if needed
				intake.stop();

				lights.setColor(ColorCode.NoteInsideRobot);
			} else if (indexer.indexingNoteState == IndexerState.NoteTooIntakeExtreme) {
				indexer.rawIndexer(-Constants.Indexer.kSpinSpeed);
				intake.intake(true);

				lights.setColor(ColorCode.NoteInsideRobot);
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
		limelight.setLEDMode_ForceOff();
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