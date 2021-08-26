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
	private Joystick second = new Joystick(Constants.SECOND_DRIVER_JOYSTICK);

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

	private final Command triggerTurboCommand = new RunCommand(
		() -> triggerTurbo(),
		Robot.drive
	);

	Command[] triggerCommands = {triggerFlywheelOutCommand, triggerTurboCommand};

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
		JoystickButton invertButton = new JoystickButton(controller, Constants.BUTTON_START);
		invertButton.whenPressed(new InstantCommand(
			()-> Robot.drive.toggleDriveMode(),
			Robot.drive)
		);


		//Button bindings for Driver here
		JoystickButton intakeIn = new JoystickButton(controller, Constants.BUTTON_RB);
		intakeIn.whileHeld(CommandFactory.intakeInCommand());
		intakeIn.whenReleased(CommandFactory.intakeStopCommand());

		JoystickButton intakeOut = new JoystickButton(controller, Constants.BUTTON_LB);
		intakeOut.whileHeld(CommandFactory.intakeOutCommand());
		intakeOut.whenReleased(CommandFactory.intakeStopCommand());

		// JoystickButton intakeOut2 = new JoystickButton(rightStick, Constants.BUTTON_5);
		// intakeOut2.whileHeld(CommandFactory.intakeOutCommand());
		// intakeOut2.whenReleased(CommandFactory.intakeStopCommand());

		JoystickButton turretWheelIn = new JoystickButton(controller, Constants.BUTTON_X);
		turretWheelIn.whileHeld(CommandFactory.turretWheelInCommand());
		turretWheelIn.whenReleased(CommandFactory.turretWheelStopCommand());

		JoystickButton turretWheelOut = new JoystickButton(controller, Constants.BUTTON_B);
		turretWheelOut.whileHeld(CommandFactory.turretWheelOutCommand());
		turretWheelOut.whenReleased(CommandFactory.turretWheelStopCommand());

		//JoystickButton flywheelIn = new JoystickButton(leftStick, Constants.BUTTON_5);
		//flywheelIn.whileHeld(CommandFactory.flywheelInCommand());
		//flywheelIn.whenReleased(CommandFactory.flywheelStopCommand());

		// old code for the flywheel out button
		// JoystickButton flywheelOut = new JoystickButton(controller, BUTTON_RT;
		// flywheelOut.whileHeld(CommandFactory.flywheelOutCommand());
		// flywheelOut.whenReleased(CommandFactory.flywheelStopCommand());

		// JoystickButton turretSwivel = new JoystickButton(controller, Constants.BUTTON_12);
		// turretSwivel.whileHeld(CommandFactory.turretSwivelAuto());
		// turretSwivel.whenReleased(CommandFactory.turretSwivelStopCommand());
		
		// JoystickButton turretSwivelLeft = new JoystickButton(controller, Constants.BUTTON_9);
		// turretSwivelLeft.whileHeld(CommandFactory.turretSwivelLeftCommand());
		// turretSwivelLeft.whenReleased(CommandFactory.turretSwivelStopCommand());

		// JoystickButton turretSwivelRight = new JoystickButton(controller, Constants.BUTTON_10);
		// turretSwivelRight.whileHeld(CommandFactory.turretSwivelRightCommand());
		// turretSwivelRight.whenReleased(CommandFactory.turretSwivelStopCommand());

		// JoystickButton hookUp = new JoystickButton(controller, Constants.BUTTON_6);
		// hookUp.whileHeld(CommandFactory.hookUpCommand());
		// hookUp.whenReleased(CommandFactory.hookStopCommand());

		// JoystickButton hookDown = new JoystickButton(controller, Constants.BUTTON_4);
		// hookDown.whileHeld(CommandFactory.hookDownCommand());
		// hookDown.whenReleased(CommandFactory.hookStopCommand());

		// JoystickButton climb = new JoystickButton(controller, Constants.BUTTON_7);
		// climb.whileHeld(CommandFactory.climbCommand());
		// climb.whenReleased(CommandFactory.climbStopCommand());

		// old code for the turbo button
		// JoystickButton turboButton = new JoystickButton(controller, BUTTON_LT);
		// turboButton.whenPressed(new InstantCommand(
		// 	()-> Robot.drive.toggleTurbo(),
		// 	Robot.drive)
		// );

		// turboButton.whenReleased(new InstantCommand(
		// 	()-> Robot.drive.toggleTurbo(),
		// 	Robot.drive)
		// );
	}

	private void triggerFlywheelOut() {
		if (controller.getRawAxis(Constants.RIGHT_TRIGGER_AXIS) > 0) {
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

	private void configureDefaultCommands() {
		CommandScheduler scheduler = CommandScheduler.getInstance();

		scheduler.setDefaultCommand(Robot.drive, defaultDriveCommand);
		scheduler.schedule(triggerCommands);
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
