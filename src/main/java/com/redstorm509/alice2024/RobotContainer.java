package com.redstorm509.alice2024;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.proto.Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SerialPort.StopBits;

import com.redstorm509.alice2024.autonomous.FourNoteAmpSideFar;
import com.redstorm509.alice2024.autonomous.OneNote;
import com.redstorm509.alice2024.autonomous.OneNoteAndTaxi;
import com.redstorm509.alice2024.autonomous.SabotageAuto;
import com.redstorm509.alice2024.autonomous.TESTING;
import com.redstorm509.alice2024.autonomous.ThreeNoteAmpSide;
import com.redstorm509.alice2024.autonomous.TwoNoteAmpSide;
import com.redstorm509.alice2024.autonomous.WIPFourNoteAmpSideNear;
import com.redstorm509.alice2024.commands.*;
import com.redstorm509.alice2024.subsystems.*;
import com.redstorm509.alice2024.subsystems.Indexer.IndexerState;
import com.redstorm509.alice2024.subsystems.drive.*;
import com.redstorm509.alice2024.subsystems.vision.*;
import com.redstorm509.alice2024.util.PigeonWrapper;
import com.redstorm509.alice2024.util.drivers.REVBlinkin;
import com.redstorm509.alice2024.util.drivers.REVBlinkin.BlinkinLedMode;

import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.redstorm509.stormkit.controllers.ThrustmasterJoystick;
import com.redstorm509.stormkit.controllers.ThrustmasterJoystick.StickButton;

public class RobotContainer {
	private final PigeonWrapper pigeon = new PigeonWrapper(30, Constants.kCANIvore);

	private ThrustmasterJoystick driverLeft = new ThrustmasterJoystick(0);
	private ThrustmasterJoystick driverRight = new ThrustmasterJoystick(1);
	private CommandXboxController operator = new CommandXboxController(2);

	private final SwerveDrive swerve;
	private final Intake intake;
	public final Indexer indexer;
	public final Shooter shooter;
	private final ArmRS arm;
	private final Climber climber;
	public final REVBlinkin led;
	public final Limelight intakeCamera = new Limelight("limelight-intake");
	private final Limelight shooterCamera = new Limelight("limelight-arm");

	private SendableChooser<Command> chooser = new SendableChooser<Command>();

	public RobotContainer() {
		this.swerve = new SwerveDrive(pigeon, shooterCamera);
		this.intake = new Intake();
		this.indexer = new Indexer();
		this.shooter = new Shooter();
		this.arm = new ArmRS();
		this.climber = new Climber(pigeon);
		this.led = new REVBlinkin(9);

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

		// Binds heading locks to the left stick's dpad. Pressing up will face forward,
		// pressing down will face backward.
		(new Trigger(() -> driverRight.getPOV(0) == 0))
				.onTrue(Commands.runOnce(() -> swerve.setTargetHeading(0), swerve));
		(new Trigger(() -> driverRight.getPOV(0) == 90))
				.onTrue(Commands.runOnce(() -> swerve.setTargetHeading(-90), swerve));
		(new Trigger(() -> driverRight.getPOV(0) == 270))
				.onTrue(Commands.runOnce(() -> swerve.setTargetHeading(90), swerve));
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

		// Teleop auto commands
		driverRight.isDownBind(StickButton.Bottom, new AutoAlign(
				swerve,
				arm,
				shooterCamera,
				() -> nonInvSquare(-driverLeft.getY()),
				() -> nonInvSquare(-driverLeft.getX()),
				() -> nonInvSquare(-driverRight.getX())));

		driverLeft.isDownBind(StickButton.Bottom, new AutoPickup(
				swerve,
				intakeCamera,
				intake,
				indexer,
				() -> nonInvSquare(-driverLeft.getY()),
				() -> nonInvSquare(-driverLeft.getX()),
				() -> nonInvSquare(-driverRight.getX())));

		// Basic intake and outake commands
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
			SmartDashboard.putBoolean("Is At Shoot Speed", shooter.isAtShooterVelocity());
			SmartDashboard.putBoolean("Is Winding Up", true);
		}, () -> {
			shooter.setShooterVelocity(0.0);
			SmartDashboard.putBoolean("Is At Shoot Speed", false);
			SmartDashboard.putBoolean("Is Winding Up", false);
		}, shooter));

		operator.a().onTrue(new SetPivot(arm, 43));
		operator.y().onTrue(new SetPivot(arm, Constants.Arm.kMinPivot + 10));

		// When the B button is held down, the arm goes into raw output mode; there are
		// no safeties.
		// operator.getHID().getBButton()
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
		chooser.addOption("Four Note (Far) [AMP SIDE]", new FourNoteAmpSideFar(swerve, shooter, arm, indexer, intake));
		chooser.addOption("Four Note (Close) [AMP SIDE]",
				new WIPFourNoteAmpSideNear(swerve, shooter, arm, indexer, intake));
		chooser.addOption("Three Note (Close) [AMP SIDE]", new ThreeNoteAmpSide(swerve, shooter, arm, indexer, intake));
		chooser.addOption("Two Note (Close) [AMP SIDE]", new TwoNoteAmpSide(swerve, shooter, arm, indexer, intake));
		chooser.addOption("One Note [ANY]", new OneNote(swerve, shooter, arm, indexer, intake));
		chooser.addOption("One Note and Taxi [SOURCE SIDE]", new OneNoteAndTaxi(swerve, shooter, arm, indexer, intake));
		chooser.addOption("Sabotage / The Samin Special", new SabotageAuto(swerve));
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

		// indexer.indexingNoteState = IndexerState.HasNote;
	}

	public void onTeleopEntry() {
	}
}