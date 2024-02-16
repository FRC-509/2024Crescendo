package com.redstorm509.alice2024.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.redstorm509.alice2024.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class PivotShooter extends Command {
	private final Shooter shooter;
	private final DoubleSupplier zoomyZoom;

	public PivotShooter(Shooter shooter, DoubleSupplier zoomyZoom) {
		this.shooter = shooter;
		this.zoomyZoom = zoomyZoom;
		addRequirements(shooter);
	}

	@Override
	public void execute() {
		shooter.setPivotOutput(zoomyZoom.getAsDouble());
	}
}