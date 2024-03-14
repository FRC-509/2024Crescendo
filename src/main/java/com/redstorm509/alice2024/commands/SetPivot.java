package com.redstorm509.alice2024.commands;

import com.redstorm509.alice2024.subsystems.ArmIS;
import com.redstorm509.alice2024.subsystems.ArmIS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

public class SetPivot extends Command {
	private ArmIS arm;
	private double targetAngle;

	public SetPivot(ArmIS arm, double targetAngle) {
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
		return MathUtil.isNear(targetAngle, arm.getPivotDegrees(), 10.0);
	}
}
