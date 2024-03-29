package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.CommandFactory;
import frc.robot.commands.Direction;

public class RobotContainer {
	// Initialize the driver controls
	private Joystick controller = new Joystick(Constants.DRIVER_CONTROLLER);

	//Initialize second driver controls
	private Joystick secondController = new Joystick(Constants.SECOND_DRIVER_CONTROLLER);

	// Initialize the drive command
	private final Command defaultDriveCommand = new RunCommand(
		() -> Robot.drive.tank(
			this.controller.getRawAxis(Constants.LEFT_STICK_AXIS),
			this.controller.getRawAxis(Constants.RIGHT_STICK_AXIS)
		),
		Robot.drive
	);

	private final Command triggerFlywheelOutCommand = new RunCommand(
		() -> triggerFlywheelOut(),
		Robot.shooter
	);

	private final Command driveCircleCommand = new RunCommand(
		() -> Robot.drive.driveCircle(.5, 360, Direction.RIGHT, 48),
		Robot.drive
		);

	private final Command invertDriveCommand = new RunCommand(
		() -> Robot.drive.tank(
			-this.controller.getRawAxis(Constants.LEFT_STICK_AXIS),
			-this.controller.getRawAxis(Constants.RIGHT_STICK_AXIS)
		),
		Robot.drive
	);

	private final Command circleDriveCommand = new SequentialCommandGroup(
		CommandFactory.driveCircleCommand(0.2, 180, Direction.RIGHT, 24)
	);

	private final Command driveForwardCommand = new SequentialCommandGroup(
		CommandFactory.driveDistanceCommand(60, Direction.FORWARD)
	);

	private final Command turnLeftCommand = new SequentialCommandGroup(
		CommandFactory.angleTurnCommand(0.5, 90, Direction.LEFT)
	);

	private final Command slalomCommand = CommandFactory.slalomAutoCommand();
	private final Command barrelCommand = CommandFactory.barrelAutoCommand();
	private final Command offLineCommand = CommandFactory.driveOffLineCommand();
	private final Command ballShootCommand = CommandFactory.driveShootAutoCommand();

	//creates a hashMap

	private Map<String, Command> commands = Map.ofEntries(
		Map.entry("shootBall",ballShootCommand)
	);

	/**
	 * The container for the robot.  Contains subsystems, OI devices, and
	 * commands.
	 */
	public RobotContainer() {
		// Configure the button bindings
		configureButtonBindings();

		// Configure the default commands
		configureDefaultCommands();
	}

	/**
	 * Use this method to define your button->command mappings.  Buttons can be
	 * created by instantiat ing a {@link GenericHID} or one of its subclasses
	 * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and
	 * then passing it to a
	 * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {
		JoystickButton invertButton = new JoystickButton(controller, Constants.BUTTON_BACK);
		invertButton.whenPressed(new InstantCommand(
			()-> Robot.drive.toggleDriveMode(),
			Robot.drive)
		);


		//Button bindings for Driver here
		JoystickButton intakeIn = new JoystickButton(controller, Constants.BUTTON_RB);
		intakeIn.whileHeld(CommandFactory.intakeInCommand());
		intakeIn.whenReleased(CommandFactory.intakeStopCommand());

		JoystickButton intakeIn2 = new JoystickButton(secondController, Constants.BUTTON_RB);
		intakeIn2.whileHeld(CommandFactory.intakeInCommand());
		intakeIn2.whenReleased(CommandFactory.intakeStopCommand());

		JoystickButton intakeOut = new JoystickButton(controller, Constants.BUTTON_LB);
		intakeOut.whileHeld(CommandFactory.intakeOutCommand());
		intakeOut.whenReleased(CommandFactory.intakeStopCommand());

		JoystickButton intakeOut2 = new JoystickButton(secondController, Constants.BUTTON_LB);
		intakeOut2.whileHeld(CommandFactory.intakeOutCommand());
		intakeOut2.whenReleased(CommandFactory.intakeStopCommand());

		JoystickButton turretWheelIn = new JoystickButton(secondController, Constants.BUTTON_X);
		turretWheelIn.whileHeld(CommandFactory.turretWheelInCommand());
		turretWheelIn.whenReleased(CommandFactory.turretWheelStopCommand());

		JoystickButton turretWheelOut = new JoystickButton(secondController, Constants.BUTTON_B);
		turretWheelOut.whileHeld(CommandFactory.turretWheelOutCommand());
		turretWheelOut.whenReleased(CommandFactory.turretWheelStopCommand());

		//JoystickButton flywheelIn = new JoystickButton(leftStick, Constants.BUTTON_5);
		//flywheelIn.whileHeld(CommandFactory.flywheelInCommand());
		//flywheelIn.whenReleased(CommandFactory.flywheelStopCommand());

		JoystickButton climb = new JoystickButton(secondController, Constants.BUTTON_START);
		climb.whileHeld(CommandFactory.climbCommand());
		climb.whenReleased(CommandFactory.climbStopCommand());

		JoystickButton autoTurretSwivel = new JoystickButton(secondController, Constants.BUTTON_A);
		autoTurretSwivel.whenPressed(CommandFactory.turretSwivelAuto());
	}

	private void triggerFlywheelOut() {
		if (secondController.getRawAxis(Constants.RIGHT_TRIGGER_AXIS) > 0) {
			Robot.shooter.flywheelOut();
		}
		else {
			Robot.shooter.flywheelStop();
		}
	}

	private void triggerTurbo() {
		if (controller.getRawAxis(Constants.LEFT_TRIGGER_AXIS) > 0) {
			Robot.drive.setTurbo(true);
		}
		else {
			Robot.drive.setTurbo(false);
		}
	}

	private void hook() {
		if (secondController.getPOV() == 0) {
			Robot.climber.hookUp();
		}
		else if (secondController.getPOV() == 180) {
			Robot.climber.hookDown();
		}
		else {
			Robot.climber.hookStop();
		}
	}

	private void turretSwivel() {
		if (secondController.getPOV() == 270) {
			Robot.shooter.turretSwivelLeft();
		}
		else if (secondController.getPOV() == 90) {
			Robot.shooter.turretSwivelRight();
		}
		else {
			Robot.shooter.turretSwivelStop();
		}
	}

	private void configureDefaultCommands() {
		CommandScheduler scheduler = CommandScheduler.getInstance();

		scheduler.setDefaultCommand(Robot.drive, defaultDriveCommand);
		scheduler.setDefaultCommand(Robot.shooter, triggerFlywheelOutCommand);
		scheduler.addButton(
			() -> triggerTurbo()
		);
		scheduler.addButton(
			() -> hook()
		);
		scheduler.addButton(
			() -> turretSwivel()
		);
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand(String key) {
		// An ExampleCommand will run in autonomous
		Command command = this.commands.get(key);
		if(command == null){
			command = this.ballShootCommand;
		}
		return command;
	}
}
