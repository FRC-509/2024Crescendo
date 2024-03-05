package com.redstorm509.alice2024.commands;

import java.util.function.DoubleSupplier;
import com.redstorm509.alice2024.Constants;
import com.redstorm509.alice2024.subsystems.Intake;
import com.redstorm509.alice2024.subsystems.Shooter;
import com.redstorm509.alice2024.subsystems.Shooter.IndexerState;
import com.redstorm509.alice2024.subsystems.drive.SwerveDrive;
import com.redstorm509.alice2024.subsystems.vision.Limelight;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoPickupExperimental extends Command {

	private SwerveDrive swerve;
	private Limelight limelight;
	private Intake intake;
	private Shooter shooter;
	private boolean beganIntaking;
	private DoubleSupplier xSupplier;
	private DoubleSupplier ySupplier;
	private DoubleSupplier rotationSupplier;

	public AutoPickupExperimental(
			SwerveDrive swerve,
			Limelight limelight,
			Intake intake,
			Shooter shooter,
			DoubleSupplier xSupplier,
			DoubleSupplier ySupplier,
			DoubleSupplier rotationSupplier) {
		this.swerve = swerve;
		this.limelight = limelight;
		this.intake = intake;
		this.shooter = shooter;

		this.xSupplier = xSupplier;
		this.ySupplier = ySupplier;
		this.rotationSupplier = rotationSupplier;

		addRequirements(swerve, intake);
	}

	@Override
	public void initialize() {
		beganIntaking = false;

		limelight.setLEDMode_ForceBlink();
		limelight.setPipelineIndex(Constants.Vision.Pipeline.NeuralNetwork);
	}

	@Override
	public void execute() {
		if (!limelight.getTV() && !beganIntaking) {
			limelight.setLEDMode_ForceOn();
			swerve.drive(new Translation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble()).times(Constants.kMaxSpeed),
					rotationSupplier.getAsDouble(),
					false,
					false);
		} else {
			limelight.setLEDMode_ForceBlink();
		}

		// Finds distance to target and how much to move
		double angleToTarget = -limelight.getTY() + -Constants.Vision.kIntakeCameraAngleOffset;
		double distanceToTarget = Constants.Vision.kIntakeCameraHeightFromGround
				/ Math.tan(Math.toRadians(angleToTarget));
		// double check correct sign, double negatives l
		double outputMove = MathUtil.clamp(-distanceToTarget * 2, -Constants.kMaxSpeed, Constants.kMaxSpeed);

		if (limelight.getTV()) {
			swerve.drive(new Translation2d(0.0, // possibly change 0.0 to: -distanceToTarget * getTX() or scale somehow
					outputMove),
					Math.toRadians(distanceToTarget * limelight.getTX() * 1.5), // tune this
					false, false);
		}

		if (distanceToTarget < 2 || beganIntaking) {
			beganIntaking = true;

			// has note logic using beam breaks
			IndexerState indexerState = shooter.indexingNoteState();

			if (indexerState == IndexerState.Noteless) {
				shooter.rawIndexer(-Constants.Shooter.kIndexerSpinSpeed);
				intake.intake(true);
			} else if (indexerState == IndexerState.NoteTooShooter) {
				shooter.rawIndexer(Constants.Shooter.kIndexerSpinSpeed * 0.5); // increase if needed
				intake.stop();
			} else if (indexerState == IndexerState.NoteTooShooterExtreme) {
				shooter.rawIndexer(Constants.Shooter.kIndexerSpinSpeed);
				intake.stop();
			} else if (indexerState == IndexerState.NoteTooIntake) {
				shooter.rawIndexer(-Constants.Shooter.kIndexerSpinSpeed * 0.5); // increase if needed
				intake.stop();
			} else if (indexerState == IndexerState.NoteTooIntakeExtreme) {
				shooter.rawIndexer(-Constants.Shooter.kIndexerSpinSpeed);
				intake.intake(true);
			}
		}

		SmartDashboard.putNumber("Angle To Target", angleToTarget);
		SmartDashboard.putNumber("Distance From Target", distanceToTarget);
	}

	@Override
	public boolean isFinished() {
		return shooter.indexingNoteState() == IndexerState.HasNote;
	}

	@Override
	public void end(boolean wasInterrupted) {
		swerve.drive(new Translation2d(0, 0), 0, true, false);
		limelight.setLEDMode_ForceOff();
		intake.stop();
		shooter.rawIndexer(0.0);
	}
}