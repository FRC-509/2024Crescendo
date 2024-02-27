package com.redstorm509.alice2024.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.redstorm509.alice2024.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.Command;

public class DefaultClimbCommand extends Command {
	private final Climber climber;
	private final DoubleSupplier leftSpeedSup;
	private final DoubleSupplier rightSpeedSup;
	private final BooleanSupplier signSup;

	public DefaultClimbCommand(Climber climber, DoubleSupplier leftSpeedSup, DoubleSupplier rightSpeedSup,
			BooleanSupplier signSup) {
		this.climber = climber;
		this.leftSpeedSup = leftSpeedSup;
		this.rightSpeedSup = rightSpeedSup;
		this.signSup = signSup;
		addRequirements(climber);
	}

	@Override
	public void execute() {
		double sign = signSup.getAsBoolean() ? 1.0d : -1.0d;
		climber.leftClimb(sign * leftSpeedSup.getAsDouble());
		climber.rightClimb(sign * rightSpeedSup.getAsDouble());
	}
}