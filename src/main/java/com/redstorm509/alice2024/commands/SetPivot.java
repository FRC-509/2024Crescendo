package com.redstorm509.alice2024.commands;

import org.ejml.equation.Symbol;

import com.redstorm509.alice2024.subsystems.Arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

public class SetPivot extends Command {
	private Arm arm;
	private double targetAngle;

	public SetPivot(Arm arm, double targetAngle) {
		this.arm = arm;
		this.targetAngle = targetAngle;
		addRequirements(arm);
	}

	@Override
	public void initialize() {
		arm.setPivotDegrees(targetAngle);
	}

	@Override
	public boolean isFinished() {
		System.out.println("Pivot: " + arm.getPivotDegrees() + " Target: " + targetAngle);
		return MathUtil.isNear(targetAngle, arm.getPivotDegrees(), 10.0);
	}
}
