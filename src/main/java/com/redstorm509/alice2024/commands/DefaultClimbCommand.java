package com.redstorm509.alice2024.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.redstorm509.alice2024.Constants;
import com.redstorm509.alice2024.subsystems.Climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

public class DefaultClimbCommand extends Command {

	private Climber climber;
	private DoubleSupplier extendSupplier;
	private DoubleSupplier retractSupplier;
	private BooleanSupplier leftOnlySupplier;
	private BooleanSupplier rightOnlySupplier;
	private BooleanSupplier toggleLockSupplier;
	private Pigeon2 pigeon;

	public DefaultClimbCommand(
			Climber climber,
			DoubleSupplier extendSupplier,
			DoubleSupplier retractSupplier,
			BooleanSupplier leftOnlySupplier,
			BooleanSupplier rightOnlySupplier,
			BooleanSupplier toggleLockSupplier,
			Pigeon2 pigeon) {
		this.climber = climber;
		this.extendSupplier = extendSupplier;
		this.retractSupplier = retractSupplier;
		this.leftOnlySupplier = leftOnlySupplier;
		this.rightOnlySupplier = rightOnlySupplier;
		this.toggleLockSupplier = toggleLockSupplier;
		this.pigeon = pigeon;

		addRequirements(climber);
	}

	@Override
	public void initialize() {
		climber.unlockLeft();
		climber.unlockRight();
	}

	@Override
	public void execute() {
		climber.unlockLeft();
		climber.unlockRight();

		if (toggleLockSupplier.getAsBoolean()) {
			climber.toggleLockLeft();
			climber.toggleLockRight();
		}
		boolean extending = Math.abs(extendSupplier.getAsDouble()) > 0.1;
		boolean retracting = Math.abs(retractSupplier.getAsDouble()) > 0.1;

		double roll = pigeon.getRoll().getValueAsDouble() - climber.getBootRoll();

		double rollCompensation = MathUtil.clamp(Math.abs(roll), 0.0, Constants.Climber.kMaxRollCompensationAngle)
				/ Constants.Climber.kMaxRollCompensationAngle;

		if (extending || retracting) {
			double output = extending ? extendSupplier.getAsDouble() : -retractSupplier.getAsDouble();
			double compensatedOutput = extending ? extendSupplier.getAsDouble() - rollCompensation
					: -retractSupplier.getAsDouble() + rollCompensation;

			// confirm which side is positive roll
			if (leftOnlySupplier.getAsBoolean()) {
				climber.leftClimb(output);
			} else if (rightOnlySupplier.getAsBoolean()) {
				climber.rightClimb(output);
			} else {
				if (roll > 0) {
					climber.leftClimb(output);
					climber.rightClimb(compensatedOutput);
				} else {
					climber.leftClimb(compensatedOutput);
					climber.rightClimb(output);
				}
			}
		} else {
			// confirm which side is positive roll
			if (roll > 0) {
				climber.leftClimb(-rollCompensation);
				climber.rightClimb(0.0);
			} else {
				climber.leftClimb(0.0);
				climber.rightClimb(-rollCompensation);
			}
		}
	}
}
