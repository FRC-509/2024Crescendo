package com.redstorm509.alice2024.commands;

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
	private Pigeon2 pigeon;

	public DefaultClimbCommand(
			Climber climber,
			DoubleSupplier extendSupplier,
			DoubleSupplier retractSupplier,
			Pigeon2 pigeon) {
		this.climber = climber;
		this.extendSupplier = extendSupplier;
		this.retractSupplier = retractSupplier;
		this.pigeon = pigeon;

		addRequirements(climber);
	}

	@Override
	public void execute() {
		boolean extending = Math.abs(extendSupplier.getAsDouble()) > 0.1;
		boolean retract = Math.abs(retractSupplier.getAsDouble()) > 0.1;

		double roll = pigeon.getRoll().getValueAsDouble();

		double rollCompensation = MathUtil.clamp(Math.abs(roll), 0.0, Constants.Climber.kMaxRollCompensationAngle)
				/ Constants.Climber.kMaxRollCompensationAngle;

		if (extending || retract) {
			double output = extending ? extendSupplier.getAsDouble() : -retractSupplier.getAsDouble();
			double compensatedOutput = extending ? extendSupplier.getAsDouble() - rollCompensation
					: -retractSupplier.getAsDouble() + rollCompensation;

			// confirm which side is positive roll
			if (roll > 0) {
				climber.leftClimb(output);
				climber.rightClimb(compensatedOutput);
			} else {
				climber.leftClimb(compensatedOutput);
				climber.rightClimb(output);
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
