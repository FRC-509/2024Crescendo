package com.redstorm509.alice2024;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotBase;

import com.redstorm509.alice2024.autonomous.*;
import com.redstorm509.alice2024.commands.*;
import com.redstorm509.alice2024.subsystems.*;
import com.redstorm509.alice2024.subsystems.Indexer.IndexerState;
import com.redstorm509.alice2024.subsystems.drive.*;
import com.redstorm509.alice2024.subsystems.vision.*;
import com.redstorm509.alice2024.util.PigeonWrapper;
import com.redstorm509.alice2024.util.drivers.REVBlinkin;
import com.redstorm509.stormkit.controllers.ThrustmasterJoystick;
import com.redstorm509.stormkit.controllers.ThrustmasterJoystick.StickButton;

public class RobotContainer {
	private final PigeonWrapper pigeon = new PigeonWrapper(30, Constants.kCANIvore);

	private ThrustmasterJoystick driverLeft = new ThrustmasterJoystick(0);
	private ThrustmasterJoystick driverRight = new ThrustmasterJoystick(1);
	private CommandXboxController operator = new CommandXboxController(2);

	private final SwerveDrive swerve;
	private final Intake intake;
	private final Indexer indexer;
	private final Shooter shooter;
	private final Arm arm;
	private final Climber climber;
	private final REVBlinkin lights;
	private final Limelight intakeCamera = new Limelight("limelight-intake");
	private final Limelight shooterCamera = new Limelight("limelight-arm");

	private SendableChooser<Command> chooser = new SendableChooser<Command>();

	public RobotContainer() {
		this.swerve = new SwerveDrive(pigeon, shooterCamera);
		this.intake = new Intake();
		this.shooter = new Shooter();
		this.arm = new Arm();
		this.indexer = new Indexer(() -> arm.armIsDown());
		this.climber = new Climber(pigeon);
		this.lights = new REVBlinkin(9);

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
				() -> true));

		// Binds heading locks to the right stick's dpad. Pressing up will face forward,
		// pressing down will face backward.
		(new Trigger(() -> driverRight.getPOV(0) == 0))
				.onTrue(Commands.runOnce(() -> swerve.setTargetHeading(0), swerve));
		(new Trigger(() -> driverRight.getPOV(0) == 90))
				.onTrue(Commands.runOnce(() -> swerve.setTargetHeading(-90), swerve));
		(new Trigger(() -> driverRight.getPOV(0) == 270))
				.onTrue(Commands.runOnce(() -> swerve.setTargetHeading(90), swerve));
		(new Trigger(() -> driverRight.getPOV(0) == 180))
				.onTrue(Commands.runOnce(() -> swerve.setTargetHeading(180), swerve));

		(new Trigger(() -> driverLeft.getPOV(0) == 0))
				.onTrue(Commands.runOnce(() -> {
					indexer.setNoteless();
					arm.setArmIsDown();
				}, indexer, arm));

		// Toggle heading correction by pressing the bottom-rightmost botton on the left
		// side of the right stick. Heading correction defaults to ON at boot.
		driverRight.isPressedBind(StickButton.LeftSideRightBottom,
				Commands.runOnce(() -> swerve.toggleHeadingCorrection(), swerve));

		// Zeroes the gyroscope when the bottom button the left stick is pressed.
		driverLeft.isPressedBind(StickButton.Left, Commands.runOnce(() -> {
			pigeon.setYaw(0);
			swerve.setTargetHeading(0);
		}, swerve));

		// Teleop auto commands
		driverRight.isDownBind(StickButton.Bottom, new AutoAlign(
				swerve,
				arm,
				shooterCamera,
				lights,
				() -> nonInvSquare(-driverLeft.getY()),
				() -> nonInvSquare(-driverLeft.getX()),
				() -> nonInvSquare(-driverRight.getX())));

		driverLeft.isDownBind(StickButton.Bottom, new AutoPickup(
				swerve,
				intakeCamera,
				intake,
				indexer,
				lights,
				() -> nonInvSquare(-driverLeft.getY()),
				() -> nonInvSquare(-driverLeft.getX()),
				() -> nonInvSquare(-driverRight.getX())));

		// Basic intake and outake commands
		driverLeft.isDownBind(StickButton.Trigger, new IntakeNote(intake, indexer, arm, lights));
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
					lights.setDefault();
				},
				intake, indexer, lights));

		// raw indexer
		driverLeft.isDownBind(StickButton.Right, Commands.startEnd(() -> {
			indexer.rawIndexer(-Constants.Indexer.kSpinSpeed / 2);
		}, () -> {
			indexer.rawIndexer(0);
		}, indexer));
		driverRight.isDownBind(StickButton.Left, Commands.startEnd(() -> {
			indexer.rawIndexer(Constants.Indexer.kSpinSpeed / 2);
		}, () -> {
			indexer.rawIndexer(0);
		}, indexer));

		operator.rightBumper().whileTrue(Commands.runEnd(() -> {
			indexer.rawIndexer(Constants.Indexer.kShootSpeed);
			SmartDashboard.putBoolean("Is Shooting", true);
		}, () -> {
			indexer.rawIndexer(0.0);
			indexer.setNoteless();
			SmartDashboard.putBoolean("Is Shooting", false);
		}, indexer));

		operator.leftBumper().whileTrue(Commands.runEnd(() -> {
			shooter.setShooterVelocity(-Constants.Shooter.kTargetSpeed);
		}, () -> {
			shooter.setShooterVelocity(0);
		}, shooter));

		operator.leftTrigger(0.7).whileTrue(Commands.runEnd(() -> {
			shooter.setShooterVelocity(-50.9);
		}, () -> {
			shooter.setShooterVelocity(0);
		}, shooter));

		operator.a().onTrue(new ConditionalCommand(
				new SetPivot(arm, 38.84),
				new InstantCommand(),
				() -> (indexer.indexingNoteState == IndexerState.Noteless
						|| indexer.indexingNoteState == IndexerState.HasNote)));
		operator.b().onTrue(new ConditionalCommand(
				new SetPivot(arm, -47.0),
				new InstantCommand(),
				() -> (indexer.indexingNoteState == IndexerState.Noteless
						|| indexer.indexingNoteState == IndexerState.HasNote)));
		operator.y().onTrue(new SetPivot(arm, Constants.Arm.kMinPivot + 4));
		arm.setDefaultCommand(new DefaultPivotCommand(arm,
				() -> nonInvSquare(-operator.getLeftY()) / 5, () -> false));

		// The left and right buttons on the d-pad indicate that only that climber
		// should actuate. The X button toggles the solenoids between their locked and
		// unlocked position.

		climber.setDefaultCommand(new DefaultClimbCommand(climber,
				() -> MathUtil.applyDeadband(operator.getRightY(), Constants.kStickDeadband),
				() -> operator.getHID().getPOV() == 90,
				() -> operator.getHID().getPOV() == 270,
				() -> operator.getHID().getXButton(),
				pigeon,
				false));
	}

	private void addAutonomousRoutines() {
		chooser.addOption("Sabotage (DO NOT USE!)", new Sabotage(swerve, intake, indexer, shooter));
		chooser.addOption("Sprint",
				new Sprint(swerve, arm, intake, indexer, shooter, shooterCamera, lights));

		chooser.addOption("[AMP/SOURCE] 1 Note", new ShootOneNote(swerve, shooter, arm, indexer, intake, lights));
		chooser.addOption("[SOURCE] 1 Note + Taxi", new S1CloseTaxi(swerve, shooter, arm, indexer, intake, lights));
		chooser.addOption("[AMP] 2 Note Close",
				new A2Close(swerve, shooter, arm, indexer, intake, lights));
		chooser.addOption("[AMP] 3 Note Close",
				new A3Close(swerve, shooter, arm, indexer, intake, lights));
		chooser.addOption("[AMP] 4 Note Close",
				new A4Close(swerve, shooter, arm, indexer, intake, lights));
		chooser.addOption("[SOURCE] 2 Note Close",
				new S2Close(swerve, shooter, arm, indexer, intake, lights));
		chooser.addOption("[SOURCE] 3 Note Close",
				new S3Close(swerve, shooter, arm, indexer, intake, lights));
		chooser.addOption("[SOURCE] 4 Note Close",
				new S4Close(swerve, shooter, arm, indexer, intake, lights));
		chooser.addOption("[AMP] 2 Close 1 Far", new A2Close1Midfield(swerve, shooter, arm, indexer, intake, lights));

		chooser.addOption("\"Go AFK\" (Null)", new InstantCommand());
		SmartDashboard.putData("Auto Mode", chooser);

		if (RobotBase.isSimulation()) {
			SmartDashboard.putData("Reset Swerve", Commands.runOnce(swerve::resetSimState, swerve));
		}
	}

	public Command getAutonomousCommand() {
		return chooser.getSelected();
	}

	public void onRobotEnable() {
		arm.onRobotEnable();
		pigeon.onEnable();

		intakeCamera.setLEDMode_ForceOff();
		shooterCamera.setLEDMode_ForceOff();
		lights.setDefault();
		lights.enableReset();
	}

	public void onTeleopEntry() {
		indexer.setNoteless();
	}
}