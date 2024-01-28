// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.redstorm509.alice2024;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import com.redstorm509.alice2024.autonomous.*;
import com.redstorm509.alice2024.commands.*;
import com.redstorm509.alice2024.subsystems.*;
import com.redstorm509.alice2024.subsystems.drive.*;
import com.redstorm509.alice2024.subsystems.vision.*;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.redstorm509.stormkit.controllers.ThrustmasterJoystick;
import com.redstorm509.stormkit.controllers.ThrustmasterJoystick.StickButton;
import com.redstorm509.stormkit.controllers.LogitechDualAction;

public class RobotContainer {
	private final Pigeon2 pigeon = new Pigeon2(30, Constants.kCANIvore);

	private ThrustmasterJoystick driverLeft = new ThrustmasterJoystick(0);
	private ThrustmasterJoystick driverRight = new ThrustmasterJoystick(1);
	private LogitechDualAction operator = new LogitechDualAction(2);

	private final SwerveDrive swerve;
	private final Intake intake;
	private final Shooter shooter;
	private final PreCompressor preCompressor;
	private final Limelight intakeCamera = new Limelight("limelight-front", Constants.Vision.kIntakeCameraPose);
	private final Limelight shooterCamera = new Limelight("limelight-back", Constants.Vision.kShooterCameraPose);

	private final SendableChooser<Command> chooser = new SendableChooser<Command>();

	public RobotContainer() {
		this.intake = new Intake();
		this.swerve = new SwerveDrive(pigeon);
		this.shooter = new Shooter();
		this.preCompressor = new PreCompressor();

		configureButtonBindings();
	}

	// Configure the trigger bindings
	private void configureButtonBindings() {
		swerve.setDefaultCommand(new DriveCommand(
				swerve,
				() -> driverLeft.getX(),
				() -> -driverLeft.getY(),
				() -> driverRight.getX(),
				() -> driverLeft.isDown(StickButton.Left)));
	}

	public Command getAutonomousCommand() {
		return chooser.getSelected();
	}
}