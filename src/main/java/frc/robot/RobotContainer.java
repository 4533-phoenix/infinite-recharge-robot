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
	private Joystick leftStick = new Joystick(Constants.DRIVER_JOYSTICK_LEFT);
	private Joystick rightStick = new Joystick(Constants.DRIVER_JOYSTICK_RIGHT);

	//Initialize second driver controls
	private Joystick second = new Joystick(Constants.SECOND_DRIVER_JOYSTICK);

	// Initialize the drive command
	private final Command defaultDriveCommand = new RunCommand(
		() -> Robot.drive.tank(
			this.leftStick.getRawAxis(1),
			this.rightStick.getRawAxis(1)
		),
		Robot.drive
	);

	private final Command driveCircleCommand = new RunCommand(
		() -> Robot.drive.driveCircle(.5, 360, Direction.RIGHT, 48),
		Robot.drive
		);

	private final Command invertDriveCommand = new RunCommand(
		() -> Robot.drive.tank(
			-this.leftStick.getRawAxis(1),
			-this.rightStick.getRawAxis(1)
		),
		Robot.drive
	);

	private final Command circleDriveCommand = new SequentialCommandGroup(
		CommandFactory.driveCircleCommand(0.5, 360, Direction.RIGHT, 24),
		CommandFactory.driveDistanceCommand(24, Direction.FORWARD)
	);

	private final Command driveForwardCommand = new SequentialCommandGroup(
		CommandFactory.driveDistanceCommand(72, Direction.FORWARD)
	);

	//creates a hashMap

	private Map<String, Command> commands = Map.ofEntries(
		Map.entry("driveCircleCommand", driveForwardCommand),
		Map.entry("driveDistanceCommand", driveForwardCommand)
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
		JoystickButton invertButton = new JoystickButton(leftStick, Constants.BUTTON_5);
		invertButton.whenPressed(new InstantCommand(
			()-> Robot.drive.toggleDriveMode(),
			Robot.drive)
		);


		//Button bindings for Driver here
		JoystickButton intakeIn = new JoystickButton(rightStick, Constants.TRIGGER);
		intakeIn.whileHeld(CommandFactory.intakeInCommand());
		intakeIn.whenReleased(CommandFactory.intakeStopCommand());

		JoystickButton intakeOut = new JoystickButton(rightStick, Constants.BUTTON_10);
		intakeOut.whileHeld(CommandFactory.intakeOutCommand());
		intakeOut.whenReleased(CommandFactory.intakeStopCommand());

		JoystickButton intakeOut2 = new JoystickButton(rightStick, Constants.BUTTON_5);
		intakeOut2.whileHeld(CommandFactory.intakeOutCommand());
		intakeOut2.whenReleased(CommandFactory.intakeStopCommand());

		JoystickButton turboButton = new JoystickButton(rightStick, Constants.THUMB_BUTTON);
		turboButton.whenPressed(new InstantCommand(
			()-> Robot.drive.toggleTurbo(),
			Robot.drive)
		);

		turboButton.whenReleased(new InstantCommand(
			()-> Robot.drive.toggleTurbo(),
			Robot.drive)
		);
	}

	private void configureDefaultCommands() {
		CommandScheduler scheduler = CommandScheduler.getInstance();

		scheduler.setDefaultCommand(Robot.drive, defaultDriveCommand);

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
			command = this.driveForwardCommand;
		}
		return command;
	}
}
