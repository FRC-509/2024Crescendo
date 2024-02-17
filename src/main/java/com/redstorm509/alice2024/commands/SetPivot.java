package com.redstorm509.alice2024.commands;

import com.redstorm509.alice2024.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class SetPivot extends Command {
	private Shooter shooter;
	private double targetAngle;

	public SetPivot(Shooter shooter, double targetAngle) {
		this.shooter = shooter;
		this.targetAngle = targetAngle;
		addRequirements(shooter);
	}

	@Override
	public void initialize() {
		shooter.setPivotDegrees(targetAngle);
	}

	@Override
	public boolean isFinished() {
		return Math.abs(shooter.getPivotDegrees() - targetAngle) <= 2.0;
	}
}
