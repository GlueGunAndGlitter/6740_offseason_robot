package frc.robot;

import com.ctre.phoenix6.signals.AppliedRotorPolarityValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.*;
import frc.robot.vision.AprilTagVision;
import frc.robot.vision.NoteVision;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

	// veriabels
	public static double destenceFromAprilTag = 10;

	/* Controllers */
	public static final XboxController xboxController = new XboxController(0);
	public static final CommandXboxController commandXBoxController = new CommandXboxController(0);
	private final static Joystick driver = new Joystick(0);

	/* Drive Controls */
	private static final int translationAxis = XboxController.Axis.kLeftY.value;
	private static final int strafeAxis = XboxController.Axis.kLeftX.value;
	private static final int rotationAxis = XboxController.Axis.kRightX.value;

	/* Driver Buttons */
	private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kStart.value);
	private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

	POVButton d_Uppov = new POVButton(driver, 0);
	POVButton d_Rightpov = new POVButton(driver, 90);
	POVButton d_Downpov = new POVButton(driver, 180);
	POVButton d_Leftpov = new POVButton(driver, 270);

	// private final JoystickButton IntakeEnableCommand = new JoystickButton(driver,
	// XboxController.Button.kRightBumper.value);
	/* Subsystems */
	public final static Swerve swerve = new Swerve();
	public final static Intake intake = new Intake();
	public final static Shooter shooter = new Shooter();
	public final static Kickers kickers = new Kickers();
	public final static ChangeAngelShooter changeAngelShooter = new ChangeAngelShooter();
	public final static NoteVision noteVision = new NoteVision();
	public final static AprilTagVision aprilTagVision = new AprilTagVision();
	


	private final SendableChooser<Command> autoChooser;

	/**
	 * q+
	 * `
	 * The container
	 * for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Another option that allows you to specify the default auto by its name
		// autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
		configureButtonBindings();
		setDefaultCommands();
		registerCommand();
		autoChooser = AutoBuilder.buildAutoChooser();

		Shuffleboard.getTab("Robot")
				.add("Auto", autoChooser);
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by
	 * instantiating a {@link GenericHID} or one of its subclasses ({@link
	 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
	 * it to a {@link
	 * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {

		/* Driver Buttons */
		zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroHeading()));

		commandXBoxController.a().whileTrue(intake.inputCommand()
		.alongWith(shooter.floorInputCommand()).alongWith(kickers.inputKickerCommand()));

		
		commandXBoxController.b().whileTrue(intake.outputCommand(
			Robot.IntakeRightMotorShafelbordSpeed.getDouble(0),
			Robot.IntakeLeftMotorShfelbordSpeed.getDouble(0)));

		commandXBoxController.x().whileTrue(shooter.shootUpCommand()
		.alongWith(swerve.crossWheelsCommand())
		.alongWith(new WaitCommand(3)
		.andThen(kickers.outputKickerCommand())));

		commandXBoxController.y().toggleOnTrue(changeAngelShooter.setRotationCommand());

		commandXBoxController.back().onTrue(changeAngelShooter.zeroEncodersCommand());

		commandXBoxController.rightBumper().whileTrue(changeAngelShooter.setToZeroRotationCommand());

		commandXBoxController.leftBumper().whileTrue(shooter.inputCommand()
		.alongWith(swerve.crossWheelsCommand())
		.alongWith(kickers.inputKickerCommand()));
		
		commandXBoxController.rightTrigger().onTrue(changeAngelShooter.setAngaleCommand());

	}



	private void setDefaultCommands() {
		swerve.setDefaultCommand(teleopSwerve(false));
		intake.setDefaultCommand(intake.stopMotorsCommand());
		shooter.setDefaultCommand(shooter.stopMotorsCommand());
		kickers.setDefaultCommand(kickers.stopMotorsCommand());
		changeAngelShooter.setDefaultCommand(changeAngelShooter.stopMotorsCommand());
		

	}

	private Command teleopSwerve(boolean crossWhileNotMoving) {
		return new TeleopSwerve(

				swerve,
				() -> -driver.getRawAxis(translationAxis),
				() -> -driver.getRawAxis(strafeAxis),
				() -> -driver.getRawAxis(rotationAxis),
				() -> robotCentric.getAsBoolean(),
				crossWhileNotMoving);
	}

	public void registerCommand() {

	}


	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}

	public static boolean areJoysticksMoving() {
		return Math.abs(MathUtil.applyDeadband(driver.getRawAxis(translationAxis), Constants.stickDeadband)) > 0.0 ||
				Math.abs(MathUtil.applyDeadband(driver.getRawAxis(strafeAxis), Constants.stickDeadband)) > 0.0 ||
				Math.abs(MathUtil.applyDeadband(driver.getRawAxis(rotationAxis), Constants.stickDeadband)) > 0.0;

	}
}
