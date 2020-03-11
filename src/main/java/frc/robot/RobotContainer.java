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

	private final Command invertDriveCommand = new RunCommand(
		() -> Robot.drive.tank(
			-this.leftStick.getRawAxis(1),
			-this.rightStick.getRawAxis(1)
		),
		Robot.drive
	);

	// Since the default command must never 'end' we will use a RunCommand as it
	// does not have an 'end' condition by default.
	private final Command defaultConveyorCommand = new RunCommand(
		() -> {
			// If a power cell is ready for the conveyor OR if the conveyor is
			// currently ingesting a powercell, then we need continue stepping
			// the conveyor forward. Otherwise, we need to ensure that the
			// conveyor is not moving and that the step position is zero.
			if (Robot.conveyor.ready() ||
			   (Robot.conveyor.isActive() && !Robot.conveyor.isStepComplete())) {
				Robot.conveyor.forward(0.75);
			} else {
				Robot.conveyor.stop();
				Robot.conveyor.reset();
			}
		},
		Robot.conveyor
	);

	private SequentialCommandGroup crossLineAuto = new SequentialCommandGroup(
		CommandFactory.driveDistanceCommand(24, Direction.FORWARD)
	);

	private SequentialCommandGroup goalScoreTrenchAuto = new SequentialCommandGroup(
		//new WaitCommand(5),
		CommandFactory.driveDistanceCommand(136, Direction.BACKWARD),
		new WaitCommand(1),
		// CommandFactory.emptyConveyorCommand(),
		CommandFactory.angleTurnCommand(0.35, 38.33, Direction.RIGHT),
		new ParallelDeadlineGroup(
			CommandFactory.driveDistanceCommand(107.88, Direction.FORWARD),
			CommandFactory.intakeInCommand()
		)
		// CommandFactory.angleTurnCommand(0.35, 38.33, Direction.RIGHT),
		// new ParallelDeadlineGroup(
		// 	CommandFactory.driveDistanceCommand(72, Direction.FORWARD),
		// 	CommandFactory.intakeInCommand()
		// )
	);

	private SequentialCommandGroup midScoreAuto = new SequentialCommandGroup(
		CommandFactory.driveDistanceCommand(33.67, Direction.BACKWARD),
		CommandFactory.angleTurnCommand(0.35, 26.69, Direction.RIGHT),
		CommandFactory.driveDistanceCommand(74.953, Direction.BACKWARD),
		CommandFactory.angleTurnCommand(0.35, 26.69, Direction.LEFT),
		CommandFactory.driveDistanceCommand(33, Direction.BACKWARD),
		CommandFactory.emptyConveyorCommand(),
		CommandFactory.driveDistanceCommand(33.67, Direction.FORWARD),
		CommandFactory.angleTurnCommand(0.35, 26.69, Direction.RIGHT),
		CommandFactory.driveDistanceCommand(74.953, Direction.FORWARD),
		CommandFactory.angleTurnCommand(0.35, 26.69, Direction.LEFT),
		CommandFactory.driveDistanceCommand(52, Direction.FORWARD)
		);

	private SequentialCommandGroup homeTrenchPickupScoreAuto = new SequentialCommandGroup(
		CommandFactory.driveDistanceCommand(24, Direction.FORWARD),
		new WaitCommand(1),
		new ParallelDeadlineGroup(
			CommandFactory.driveDistanceCommand(134.88, Direction.FORWARD),
			CommandFactory.intakeInCommand()
		)
		// CommandFactory.driveDistanceCommand(218.88, Direction.BACKWARD),
		// CommandFactory.angleTurnCommand(0.30, 59.128, Direction.LEFT),
		// CommandFactory.driveDistanceCommand(77.95, Direction.BACKWARD),
		// CommandFactory.angleTurnCommand(0.30, 59.128, Direction.RIGHT),
		// CommandFactory.driveDistanceCommand(1.5, Direction.BACKWARD),
		// CommandFactory.emptyConveyorCommand(),
		// CommandFactory.driveDistanceCommand(1.5, Direction.FORWARD),
		// CommandFactory.angleTurnCommand(0.30, 59.128, Direction.LEFT),
		// CommandFactory.driveDistanceCommand(77.95, Direction.FORWARD),
		// CommandFactory.angleTurnCommand(0.30, 59.128, Direction.RIGHT),
		// CommandFactory.driveDistanceCommand(90, Direction.FORWARD)
	);

	private SequentialCommandGroup awayTrenchPickupScoreAuto = new SequentialCommandGroup(
		new ParallelDeadlineGroup(
			CommandFactory.driveDistanceCommand(111.11, Direction.FORWARD),
			CommandFactory.intakeInCommand()
		),
		CommandFactory.driveDistanceCommand(111.11, Direction.BACKWARD),
		CommandFactory.angleTurnCommand(0.35, 63.97, Direction.RIGHT),
		CommandFactory.driveDistanceCommand(205.11, Direction.BACKWARD),
		CommandFactory.angleTurnCommand(0.35, 63.97, Direction.LEFT),
		CommandFactory.driveDistanceCommand(10.75, Direction.BACKWARD),
		CommandFactory.emptyConveyorCommand(),
		CommandFactory.driveDistanceCommand(140, Direction.FORWARD)
	);

	//creates a hashMap

	private Map<String, Command> commands = Map.ofEntries(
		Map.entry("trench_pickup_score", homeTrenchPickupScoreAuto),
		Map.entry("cross_initiation_line", crossLineAuto),
		Map.entry("score_trench", goalScoreTrenchAuto),
		Map.entry("score_pickup_generator", midScoreAuto),
		Map.entry("collect_trench_powercells", awayTrenchPickupScoreAuto)
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

		JoystickButton conveyorOut = new JoystickButton(rightStick, Constants.BUTTON_9);
		conveyorOut.whileHeld(CommandFactory.conveyorOutCommand());
		conveyorOut.whenReleased(CommandFactory.conveyorStopCommand());

		JoystickButton intakeOut2 = new JoystickButton(rightStick, Constants.BUTTON_5);
		intakeOut2.whileHeld(CommandFactory.intakeOutCommand());
		intakeOut2.whenReleased(CommandFactory.intakeStopCommand());

		JoystickButton conveyorOut2 = new JoystickButton(rightStick, Constants.BUTTON_3);
		conveyorOut2.whileHeld(CommandFactory.conveyorOutCommand());
		conveyorOut2.whenReleased(CommandFactory.conveyorStopCommand());

		JoystickButton conveyorIn = new JoystickButton(rightStick, Constants.BUTTON_12);
		conveyorIn.whileHeld(CommandFactory.conveyorInCommand());
		conveyorIn.whenReleased(CommandFactory.conveyorStopCommand());

		JoystickButton conveyorEmpty = new JoystickButton(leftStick, Constants.TRIGGER);
		conveyorEmpty.whileHeld(CommandFactory.emptyConveyorCommand());
		conveyorEmpty.whenReleased(CommandFactory.conveyorStopCommand());

		JoystickButton hookUp = new JoystickButton(rightStick, Constants.BUTTON_6);
		hookUp.whileHeld(CommandFactory.hookUpCommand());
		hookUp.whenReleased(CommandFactory.hookStopCommand());

		JoystickButton hookDown = new JoystickButton(rightStick, Constants.BUTTON_4);
		hookDown.whileHeld(CommandFactory.hookDownCommand());
		hookDown.whenReleased(CommandFactory.hookStopCommand());

		JoystickButton climb = new JoystickButton(leftStick, Constants.BUTTON_6);
		climb.whileHeld(CommandFactory.climbCommand());
		climb.whenReleased(CommandFactory.climbStopCommand());

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
		scheduler.setDefaultCommand(Robot.conveyor, defaultConveyorCommand);
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
			command = crossLineAuto;
		}
		return command;
	}
}
