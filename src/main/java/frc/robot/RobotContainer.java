package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.CommandFactory;
import frc.robot.commands.Direction;

public class RobotContainer {
	// Initialize the driver controls
	private Joystick leftStick = new Joystick(Constants.DRIVER_JOYSTICK_LEFT);
	private Joystick rightStick = new Joystick(Constants.DRIVER_JOYSTICK_RIGHT);

	// Initialize the drive command
	private final Command driveCommand = new RunCommand(
		() -> Robot.drive.tank(
			this.leftStick.getRawAxis(1),
			this.rightStick.getRawAxis(1)
		),
		Robot.drive
	);

	/**
	 * ingestPowerCell waits until a power cell is available to be ingested.
	 * Once one is available, the 'ready' check will be 'true'. It will then
	 * step the conveyor to 'ingest' or take in the power cell.
	 */
	private final Command ingestPowerCell = new SequentialCommandGroup(
		new WaitUntilCommand(Robot.conveyor::ready),
		new FunctionalCommand(
			() -> Robot.conveyor.reset(),
			() -> Robot.conveyor.step(),
			(interrupt) -> Robot.conveyor.stop(),
			() -> Robot.conveyor.stepComplete(),
			Robot.conveyor
		)
	).perpetually();

	private SequentialCommandGroup crossLineAuto = new SequentialCommandGroup(
		CommandFactory.driveDistanceCommand(120, Direction.BACKWARD));

	private SequentialCommandGroup goalScoreTrenchAuto = new SequentialCommandGroup(
		CommandFactory.driveDistanceCommand(101, Direction.BACKWARD),
		new WaitCommand(5),
		CommandFactory.driveDistanceCommand(120, Direction.FORWARD),
		CommandFactory.angleTurnCommand(0.35, 38.33, Direction.LEFT),
		CommandFactory.driveDistanceCommand(107.88, Direction.FORWARD),
		CommandFactory.angleTurnCommand(0.35, 38.33, Direction.RIGHT),
		CommandFactory.driveDistanceCommand(72, Direction.FORWARD)
		);

	private SequentialCommandGroup midScoreAuto = new SequentialCommandGroup(
		CommandFactory.driveDistanceCommand(33.67, Direction.BACKWARD),
		CommandFactory.angleTurnCommand(0.35, 26.69, Direction.RIGHT),
		CommandFactory.driveDistanceCommand(74.953, Direction.BACKWARD),
		CommandFactory.angleTurnCommand(0.35, 26.69, Direction.LEFT),
		CommandFactory.driveDistanceCommand(33, Direction.BACKWARD),
		new WaitCommand(5),
		CommandFactory.driveDistanceCommand(33.67, Direction.FORWARD),
		CommandFactory.angleTurnCommand(0.35, 26.69, Direction.RIGHT),
		CommandFactory.driveDistanceCommand(74.953, Direction.FORWARD),
		CommandFactory.angleTurnCommand(0.35, 26.69, Direction.LEFT),
		CommandFactory.driveDistanceCommand(52, Direction.FORWARD)
		);

	private SequentialCommandGroup homeTrenchPickupScoreAuto = new SequentialCommandGroup(
		CommandFactory.driveDistanceCommand(158.88, Direction.FORWARD),
		//add parallel command for intake while moving
		//new WaitCommand(10),
		CommandFactory.driveDistanceCommand(218.88, Direction.BACKWARD),
		new WaitCommand(1),
		CommandFactory.angleTurnCommand(0.30, 59.128, Direction.LEFT),
		CommandFactory.driveDistanceCommand(77.95, Direction.BACKWARD),
		CommandFactory.angleTurnCommand(0.30, 59.128, Direction.RIGHT),
		CommandFactory.driveDistanceCommand(1.5, Direction.BACKWARD),
		new WaitCommand(3),
		CommandFactory.driveDistanceCommand(1.5, Direction.FORWARD),
		CommandFactory.angleTurnCommand(0.30, 59.128, Direction.LEFT),
		CommandFactory.driveDistanceCommand(77.95, Direction.FORWARD),
		CommandFactory.angleTurnCommand(0.30, 59.128, Direction.RIGHT),
		CommandFactory.driveDistanceCommand(90, Direction.FORWARD)
	);

	private SequentialCommandGroup awayTrenchPickupScoreAuto = new SequentialCommandGroup(
		CommandFactory.driveDistanceCommand(111.11, Direction.FORWARD),
		CommandFactory.driveDistanceCommand(111.11, Direction.BACKWARD),
		CommandFactory.angleTurnCommand(0.35, 63.97, Direction.RIGHT),
		CommandFactory.driveDistanceCommand(205.11, Direction.BACKWARD),
		CommandFactory.angleTurnCommand(0.35, 63.97, Direction.LEFT),
		CommandFactory.driveDistanceCommand(10.75, Direction.BACKWARD),
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
		JoystickButton intakeIn = new JoystickButton(rightStick, Constants.TRIGGER);
		intakeIn.whileHeld(CommandFactory.intakeInCommand());
		intakeIn.whenReleased(CommandFactory.intakeStopCommand());

		JoystickButton intakeOut = new JoystickButton(rightStick, Constants.BUTTON_5);
		intakeOut.whileHeld(CommandFactory.intakeOutCommand());
		intakeOut.whenReleased(CommandFactory.intakeStopCommand());

		JoystickButton conveyorOut = new JoystickButton(rightStick, Constants.THUMB_BUTTON);
		conveyorOut.whileHeld(CommandFactory.conveyorOutCommand());
		conveyorOut.whenReleased(CommandFactory.conveyorStopCommand());

		JoystickButton conveyorIn = new JoystickButton(rightStick, Constants.BUTTON_3);
		conveyorIn.whileHeld(CommandFactory.conveyorInCommand());
		conveyorIn.whenReleased(CommandFactory.conveyorStopCommand());

		JoystickButton conveyorEmpty = new JoystickButton(rightStick, Constants.BUTTON_4);
		conveyorEmpty.whileHeld(CommandFactory.emptyConveyorCommand());
		conveyorEmpty.whenReleased(CommandFactory.conveyorStopCommand());
	}

	private void configureDefaultCommands() {
		CommandScheduler scheduler = CommandScheduler.getInstance();

		scheduler.setDefaultCommand(Robot.drive, driveCommand);
		scheduler.setDefaultCommand(Robot.conveyor, ingestPowerCell);
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
