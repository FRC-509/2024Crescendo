package com.redstorm509.alice2024;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.redstorm509.alice2024.commands.*;
import com.redstorm509.alice2024.subsystems.*;
import com.redstorm509.alice2024.subsystems.drive.*;
import com.redstorm509.alice2024.subsystems.vision.*;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
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
	// private final Shooter shooter;
	public final Limelight intakeCamera = new Limelight("limelight-intake", Constants.Vision.kIntakeCameraPose);
	// private final Limelight shooterCamera = new Limelight("limelight-arm",
	// Constants.Vision.kShooterCameraPose);

	private SendableChooser<Command> chooser = new SendableChooser<Command>();

	public RobotContainer() {
		// Pigeon2Configuration conf = new Pigeon2Configuration();
		// conf.MountPose.MountPoseYaw = 180.0;
		// pigeon.getConfigurator().apply(conf);

		this.intake = new Intake();
		this.swerve = new SwerveDrive(pigeon);
		// this.shooter = new Shooter();

		intakeCamera.setLEDMode_ForceOff();

		configureButtonBindings();
	}

	private void configureButtonBindings() {
		// Binds translation to the left stick, and rotation to the right stick.
		// Defaults to field-oriented drive unless the left button on the left stick is
		// held down.
		swerve.setDefaultCommand(new DriveCommand(
				swerve,
				() -> MathUtil.applyDeadband(driverLeft.getX() / 2, Constants.kStickDeadband),
				() -> MathUtil.applyDeadband(-driverLeft.getY() / 2, Constants.kStickDeadband),
				() -> MathUtil.applyDeadband(-driverRight.getX() / 2, Constants.kStickDeadband),
				() -> !driverLeft.isDown(StickButton.Left)));
		// Zeroes the gyroscope when the bottom button the left stick is pressed.
		driverLeft.isPressedBind(StickButton.Bottom, Commands.runOnce(() -> {
			pigeon.setYaw(0);
			swerve.setTargetHeading(0);
		}, swerve));
		driverLeft.isPressedBind(StickButton.Left, new AutoPickup(swerve,
				intakeCamera, intake));
		driverLeft.isDownBind(StickButton.Trigger,
				Commands.startEnd(() -> intake.intake(true), () -> intake.stop(), intake));

		chooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("Auto Mode", chooser);
	}

	public Command getAutonomousCommand() {
		return chooser.getSelected();
	}
}