package com.redstorm509.alice2024.commands;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.redstorm509.alice2024.Constants;
import com.redstorm509.alice2024.subsystems.ArmIS;
import com.redstorm509.alice2024.util.math.Conversions;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

public class AUTOPIVOTAHH extends Command {
	private ArmIS arm;
	private double targetAngle;

	public AUTOPIVOTAHH(ArmIS arm, double targetAngle) {
		this.arm = arm;
		this.targetAngle = targetAngle;
		addRequirements(arm);
	}

	@Override
	public void initialize() {
		arm.setPivotDegrees(targetAngle);
	}

	@Override
	public void execute() {
		double integratedErrorDegrees = Conversions.falconToDegrees(arm.pivotLeader.getClosedLoopError().getValue(),
				Constants.Arm.kPivotGearRatio);
		double actualError = Conversions.falconToDegrees(targetAngle - arm.getPivotDegrees(),
				Constants.Arm.kPivotGearRatio);

		// If we havent reached the setpoint and the integrated encoder is galsighting
		// us
		if (!MathUtil.isNear(targetAngle, arm.getPivotDegrees(), 0.5) && integratedErrorDegrees < 1.0d) {
			arm.pivotLeader.setControl(new DutyCycleOut(0.05 * actualError));
		}
	}

	@Override
	public boolean isFinished() {
		return MathUtil.isNear(targetAngle, arm.getPivotDegrees(), 0.5);
	}

	@Override
	public void end(boolean wasInterrupted) {
		// arm.pivotLeader.setControl(new VoltageOut(0));
		arm.pivotLeader.setControl(new PositionVoltage(arm.pivotLeader.getPosition().getValueAsDouble()));
	}
}
