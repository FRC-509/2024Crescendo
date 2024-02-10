package com.redstorm509.alice2024.commands.TempNotWorking;

import java.util.function.DoubleSupplier;

import com.redstorm509.alice2024.Constants;
import com.redstorm509.alice2024.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class PivotShooter extends Command {
	private Shooter shooter;
	private double targetPivotDegrees;
	private double degreesToTarget;
	private DoubleSupplier thetaSupplier;

	public PivotShooter(DoubleSupplier thetaSupplier, Shooter shooter, double initialTargetPivotDegrees) {
		this.shooter = shooter;
		this.targetPivotDegrees = initialTargetPivotDegrees;
		this.thetaSupplier = thetaSupplier;

		addRequirements(shooter);
	}

	@Override
	public void initialize() {
		if (targetPivotDegrees > Constants.Shooter.kMaxPivot) {
			targetPivotDegrees = Constants.Shooter.kMaxPivot;
		} else if (targetPivotDegrees < Constants.Shooter.kMinPivot) {
			targetPivotDegrees = Constants.Shooter.kMinPivot;
		}
	}

	@Override
	public void execute() {
		targetPivotDegrees += thetaSupplier.getAsDouble() * Constants.Shooter.kMaxPivotSpeed;

		if (targetPivotDegrees > Constants.Shooter.kMaxPivot) {
			targetPivotDegrees = Constants.Shooter.kMaxPivot;
		} else if (targetPivotDegrees < Constants.Shooter.kMinPivot) {
			targetPivotDegrees = Constants.Shooter.kMinPivot;
		}

		degreesToTarget = shooter.getPivotDegrees() - targetPivotDegrees;

		if (degreesToTarget > Constants.Shooter.kMaxPivotSpeed) {
			degreesToTarget = Constants.Shooter.kMaxPivotSpeed;
		} else if (degreesToTarget < -Constants.Shooter.kMaxPivotSpeed) {
			degreesToTarget = -Constants.Shooter.kMaxPivotSpeed;
		}

		shooter.setPivotOutput(degreesToTarget / Constants.Shooter.kMaxPivotSpeed);
	}

	@Override
	public boolean isFinished() {
		// idk what the allowed difference for this should be, should be a PID
		return Math.abs(shooter.getPivotDegrees() - targetPivotDegrees) <= 1; // tune
	}
}