package com.redstorm509.alice2024.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.redstorm509.alice2024.Constants;
import com.redstorm509.alice2024.subsystems.Climber;
import com.redstorm509.alice2024.util.PigeonWrapper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class DefaultClimbCommand extends Command {

	private Climber climber;
	// private DoubleSupplier extendSupplier;
	// private DoubleSupplier retractSupplier;
	private DoubleSupplier outputSupplier;
	private BooleanSupplier leftOnlySupplier;
	private BooleanSupplier rightOnlySupplier;
	private BooleanSupplier toggleLockSupplier;
	private PigeonWrapper pigeon;
	private boolean usesRollCompensation;

	private Timer toggleDelay = new Timer();

	public DefaultClimbCommand(
			Climber climber,
			// DoubleSupplier extendSupplier,
			// DoubleSupplier retractSupplier,
			DoubleSupplier outputSupplier,
			BooleanSupplier leftOnlySupplier,
			BooleanSupplier rightOnlySupplier,
			BooleanSupplier toggleLockSupplier,
			PigeonWrapper pigeon,
			boolean usesRollCompensation) {
		this.climber = climber;
		// this.extendSupplier = extendSupplier;
		// this.retractSupplier = retractSupplier;
		this.outputSupplier = outputSupplier;
		this.leftOnlySupplier = leftOnlySupplier;
		this.rightOnlySupplier = rightOnlySupplier;
		this.toggleLockSupplier = toggleLockSupplier;
		this.pigeon = pigeon;
		this.usesRollCompensation = usesRollCompensation;

		addRequirements(climber);
	}

	@Override
	public void initialize() {
		climber.unlockLeft();
		climber.unlockRight();

		toggleDelay.start();
	}

	@Override
	public void execute() {
		if (toggleLockSupplier.getAsBoolean() && toggleDelay.get() > 1.0) {
			climber.toggleLockLeft();
			climber.toggleLockRight();
			toggleDelay.reset();
		}
		double output = outputSupplier.getAsDouble() * 0.40;
		boolean extending = output > 0.1;
		boolean retracting = output < -0.1;
		// boolean extending = Math.abs(extendSupplier.getAsDouble()) > 0.1;
		// boolean retracting = Math.abs(retractSupplier.getAsDouble()) > 0.1;

		double roll = pigeon.getRoll();

		double rollCompensation = usesRollCompensation
				? MathUtil.clamp(Math.abs(roll), 0.0, Constants.Climber.kMaxRollCompensationAngle)
						/ Constants.Climber.kMaxRollCompensationAngle
				: 0.0;

		if (extending || retracting) {
			// double output = extending ? extendSupplier.getAsDouble() :
			// -retractSupplier.getAsDouble();
			// double compensatedOutput = extending ? extendSupplier.getAsDouble() -
			// rollCompensation : -retractSupplier.getAsDouble() + rollCompensation;

			double compensatedOutput = extending ? output - rollCompensation : output + rollCompensation;

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
