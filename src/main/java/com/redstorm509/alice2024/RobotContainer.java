package com.redstorm509.alice2024;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

import com.redstorm509.alice2024.autonomous.ThreeNote;
import com.redstorm509.alice2024.autonomous.TwoNoteCloseToAmp;
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
	public final Indexer indexer;
	public final Shooter shooter;
	private final Arm arm;
	private final Climber climber;
	public final Limelight intakeCamera = new Limelight("limelight-intake");
	private final Limelight shooterCamera = new Limelight("limelight-arm");

	private SendableChooser<Command> chooser = new SendableChooser<Command>();

	public RobotContainer() {
		this.swerve = new SwerveDrive(pigeon, shooterCamera);
		this.intake = new Intake();
		this.indexer = new Indexer();
		this.shooter = new Shooter();
		this.arm = new Arm();
		this.climber = new Climber(pigeon);

		intakeCamera.setLEDMode_ForceOff();
		intakeCamera.setPipelineIndex(Constants.Vision.Pipeline.NeuralNetwork);

		shooterCamera.setLEDMode_ForceOff();
		shooterCamera.setPipelineIndex(Constants.Vision.Pipeline.AprilTags);

		configureButtonBindings();
		addAutonomousRoutines();
	}

	private static double nonInvSquare(double axis) {
		double deadbanded = MathUtil.applyDeadband(axis, Constants.kStickDeadband);
		double squared = Math.abs(deadbanded) * deadbanded;
		return squared;
	}

	private void configureButtonBindings() {
		// Binds translation to the left stick, and rotation to the right stick.
		// Defaults to field-oriented drive unless the left button on the left stick is
		// held down.

		swerve.setDefaultCommand(new DefaultDriveCommand(
				swerve,
				() -> nonInvSquare(-driverLeft.getY()),
				() -> nonInvSquare(-driverLeft.getX()),
				() -> nonInvSquare(-driverRight.getX()),
				() -> !driverLeft.isDown(StickButton.Left)));

		// Binds heading locks to the left stick's dpad. Pressing up will face forward,
		// pressing down will face backward.
		(new Trigger(() -> driverRight.getPOV(0) == 0))
				.onTrue(Commands.runOnce(() -> swerve.setTargetHeading(0), swerve));
		(new Trigger(() -> driverRight.getPOV(0) == 180))
				.onTrue(Commands.runOnce(() -> swerve.setTargetHeading(180), swerve));
		// Toggle heading correction by pressing the bottom-rightmost botton on the left
		// side of the right stick. Heading correction defaults to ON at boot.
		driverRight.isPressedBind(StickButton.LeftSideRightBottom,
				Commands.runOnce(() -> swerve.toggleHeadingCorrection(), swerve));
		// Zeroes the gyroscope when the bottom button the left stick is pressed.
		driverLeft.isPressedBind(StickButton.Left, Commands.runOnce(() -> {
			pigeon.setYaw(0);
			swerve.setTargetHeading(0);
		}, swerve));

		driverRight.isDownBind(StickButton.Bottom, new AutoAlign(
				swerve,
				arm,
				shooterCamera,
				() -> nonInvSquare(-driverLeft.getY()),
				() -> nonInvSquare(-driverLeft.getX()),
				() -> nonInvSquare(-driverRight.getX())));

		driverLeft.isDownBind(StickButton.Bottom, new AutoPickupExperimental(
				swerve,
				intakeCamera,
				intake,
				indexer,
				() -> nonInvSquare(-driverLeft.getY()),
				() -> nonInvSquare(-driverLeft.getX()),
				() -> nonInvSquare(-driverRight.getX())));

		driverLeft.isDownBind(StickButton.Trigger, new IntakeNote(intake, indexer));
		driverRight.isDownBind(StickButton.Trigger, Commands.startEnd(
				() -> {
					intake.intake(false);
					// Goes OUTTTTTT
					indexer.rawIndexer(Constants.Indexer.kSpinSpeed);
				},
				() -> {
					intake.stop();
					indexer.rawIndexer(0);
					indexer.setNoteless();
				},
				intake, indexer));

		driverLeft.isDownBind(StickButton.Right, Commands.startEnd(() -> {
			indexer.rawIndexer(-Constants.Indexer.kSpinSpeed);
		}, () -> {
			indexer.rawIndexer(0);

		}, indexer));
		driverRight.isDownBind(StickButton.Left, Commands.startEnd(() -> {
			indexer.rawIndexer(Constants.Indexer.kSpinSpeed);
		}, () -> {
			indexer.rawIndexer(0);
		}, indexer));

		operator.rightBumper().whileTrue(new ShootNote(shooter, indexer));
		operator.leftBumper().whileTrue(Commands.startEnd(
				() -> indexer.rawIndexer(Constants.Indexer.kSpinSpeed),
				() -> indexer.rawIndexer(0), shooter));
		operator.a().onTrue(new SetPivot(arm, 43));
		operator.y().onTrue(new SetPivot(arm, Constants.Arm.kMinPivot));

		// When the B button is held down, the arm goes into raw output mode; there are
		// no
		arm.setDefaultCommand(new DefaultPivotCommand(arm,
				() -> nonInvSquare(-operator.getLeftY()) / 5, () -> operator.getHID().getBButton()));

		// The left and right buttons on the d-pad indicate that only that climber
		// should actuate. The X button toggles the solenoids between their locked and
		// unlocked position.

		climber.setDefaultCommand(new DefaultClimbCommand(climber,
				() -> operator.getRightTriggerAxis(),
				() -> operator.getLeftTriggerAxis(),
				() -> operator.getHID().getPOV() == 270,
				() -> operator.getHID().getPOV() == 90,
				() -> operator.getHID().getXButton(),
				pigeon));

	}

	private void addAutonomousRoutines() {
		chooser.addOption("Three Note", new ThreeNote(swerve, shooter, arm, indexer, intake));
		chooser.addOption("Two Note Amp Side", new TwoNoteCloseToAmp(swerve, shooter, arm, indexer, intake));
		chooser.addOption("One Note and Taxi",
				new SequentialCommandGroup(
						new ShootNote(shooter, indexer),
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
		if (DriverStation.isFMSAttached()) {
			swerve.setTargetHeading(0);
		} else {
			swerve.setTargetHeading(pigeon.getYaw().getValue());
		}
		climber.unlockLeft();
		climber.unlockRight();
	}
}