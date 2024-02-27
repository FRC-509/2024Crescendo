package com.redstorm509.alice2024;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotBase;

import com.redstorm509.alice2024.autonomous.ThreeNote;
import com.redstorm509.alice2024.commands.*;
import com.redstorm509.alice2024.subsystems.*;
import com.redstorm509.alice2024.subsystems.drive.*;
import com.redstorm509.alice2024.subsystems.vision.*;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.redstorm509.stormkit.controllers.ThrustmasterJoystick;
import com.redstorm509.stormkit.controllers.ThrustmasterJoystick.StickButton;

public class RobotContainer {
	private final Pigeon2 pigeon = new Pigeon2(30, Constants.kCANIvore);

	private ThrustmasterJoystick driverLeft = new ThrustmasterJoystick(0);
	private ThrustmasterJoystick driverRight = new ThrustmasterJoystick(1);
	private CommandXboxController operator = new CommandXboxController(2);

	private final SwerveDrive swerve;
	private final Intake intake;
	private final Shooter shooter;
	private final Climber climber;
	public final Limelight intakeCamera = new Limelight("limelight-intake");
	private final Limelight shooterCamera = new Limelight("limelight-arm");

	private SendableChooser<Command> chooser = new SendableChooser<Command>();

	public RobotContainer() {
		this.swerve = new SwerveDrive(pigeon);
		this.intake = new Intake();
		this.shooter = new Shooter();
		this.climber = new Climber();

		intakeCamera.setLEDMode_ForceOff();
		intakeCamera.setPipelineIndex(Constants.Vision.Pipeline.AprilTags);

		shooterCamera.setLEDMode_ForceOff();
		shooterCamera.setPipelineIndex(Constants.Vision.Pipeline.AprilTags);

		configureButtonBindings();
		addAutonomousRoutines();
	}

	private void configureButtonBindings() {
		// Binds translation to the left stick, and rotation to the right stick.
		// Defaults to field-oriented drive unless the left button on the left stick is
		// held down.

		swerve.setDefaultCommand(new DefaultDriveCommand(
				swerve,
				() -> MathUtil.applyDeadband(-driverLeft.getY(), Constants.kStickDeadband),
				() -> MathUtil.applyDeadband(-driverLeft.getX(), Constants.kStickDeadband),
				() -> MathUtil.applyDeadband(-driverRight.getX(), Constants.kStickDeadband),
				() -> !driverLeft.isDown(StickButton.Left)));
		// Zeroes the gyroscope when the bottom button the left stick is pressed.
		driverLeft.isPressedBind(StickButton.Left, Commands.runOnce(() -> {
			pigeon.setYaw(0);
			swerve.setTargetHeading(0);
		}, swerve));

		driverRight.isDownBind(StickButton.Bottom, new AimForSpeaker(swerve, shooter,
				() -> MathUtil.applyDeadband(-driverLeft.getY(), Constants.kStickDeadband),
				() -> MathUtil.applyDeadband(-driverLeft.getX(), Constants.kStickDeadband),
				() -> MathUtil.applyDeadband(-driverRight.getX(), Constants.kStickDeadband),
				shooterCamera));

		driverLeft.isDownBind(StickButton.Trigger, Commands.startEnd(
				() -> {
					// shooter.setPivotDegrees(1);
					intake.intake(true);
					// Goes INNNNNN
					shooter.rawIndexer(-Constants.Shooter.kIndexerSpinSpeed);
				},
				() -> {
					intake.stop();
					shooter.rawIndexer(0);
				},
				intake, shooter));
		driverRight.isDownBind(StickButton.Trigger, Commands.startEnd(
				() -> {
					intake.intake(false);
					// Goes OUTTTTTT
					shooter.rawIndexer(Constants.Shooter.kIndexerSpinSpeed);
				},
				() -> {
					intake.stop();
					shooter.rawIndexer(0);
				},
				intake, shooter));

		driverLeft.isDownBind(StickButton.Right, Commands.startEnd(() -> {
			shooter.rawIndexer(-Constants.Shooter.kIndexerSpinSpeed);
		}, () -> {
			shooter.rawIndexer(0);

		}, shooter));
		driverRight.isDownBind(StickButton.Left, Commands.startEnd(() -> {
			shooter.rawIndexer(Constants.Shooter.kIndexerSpinSpeed);
		}, () -> {
			shooter.rawIndexer(0);
		}, shooter));

		operator.rightBumper().whileTrue(new ShootNote(shooter, 0.5 * Constants.kFalconFreeSpeedRPS, false,
				() -> driverRight.isDown(StickButton.Right)));
		operator.leftBumper().whileTrue(Commands.startEnd(
				() -> shooter.rawIndexer(Constants.Shooter.kIndexerSpinSpeed),
				() -> shooter.rawIndexer(0), shooter));
		operator.a().onTrue(new SetPivot(shooter, 110));

		shooter.setDefaultCommand(new DefaultPivotCommand(shooter,
				() -> MathUtil.applyDeadband(-operator.getLeftY(), Constants.kStickDeadband) / 5));
		climber.setDefaultCommand(new DefaultClimbCommand(climber, () -> operator.getLeftTriggerAxis(),
				() -> operator.getRightTriggerAxis(), () -> operator.y().getAsBoolean()));
	}

	private void addAutonomousRoutines() {
		chooser.addOption("Two Note", new ThreeNote(swerve, shooter, intake));
		chooser.addOption("One Note and Taxi",
				new SequentialCommandGroup(
						new ShootNote(shooter, 0.5 * Constants.kFalconFreeSpeedRPS, true, () -> false),
						new DefaultDriveCommand(swerve, 0.7d, 0.0d, 0.0d, true).withTimeout(1)));
		chooser.addOption("Null", new InstantCommand());
		SmartDashboard.putData("Auto Mode", chooser);

		if (RobotBase.isSimulation()) {
			SmartDashboard.putData("Reset Swerve", Commands.runOnce(swerve::resetSimState, swerve));
		}
	}

	public Command getAutonomousCommand() {
		return chooser.getSelected();
	}

	public void onTeleopEntry() {
		swerve.setTargetHeading(pigeon.getYaw().getValue());
	}
}