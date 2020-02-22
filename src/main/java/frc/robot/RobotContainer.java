package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.CommandFactory;
import frc.robot.commands.Direction;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.IntakeSystem;

public class RobotContainer {
	// Initialize the robots subsystems
	private final DriveSystem driveSystem = new DriveSystem();
	private final IntakeSystem intakeSystem = new IntakeSystem();

	// Initialize the driver controls
	private Joystick leftStick = new Joystick(Constants.DRIVER_JOYSTICK_LEFT);
	private Joystick rightStick = new Joystick(Constants.DRIVER_JOYSTICK_RIGHT);

	// Initialize the drive command
	private final Command driveCommand = new RunCommand(
			() -> this.driveSystem.tank(
					this.leftStick.getRawAxis(1),
					this.rightStick.getRawAxis(1)),
					this.driveSystem
				  );

	private SequentialCommandGroup crossLineAuto = new SequentialCommandGroup(
		CommandFactory.driveDistanceCommand(120, Direction.BACKWARD, driveSystem));

	private SequentialCommandGroup goalScoreTrenchAuto = new SequentialCommandGroup(
		CommandFactory.driveDistanceCommand(101, Direction.BACKWARD, driveSystem), new WaitCommand(5),
		CommandFactory.driveDistanceCommand(120, Direction.FORWARD, driveSystem),
		CommandFactory.angleTurnCommand(0.35, 38.33, Direction.LEFT, driveSystem),
		CommandFactory.driveDistanceCommand(107.88, Direction.FORWARD, driveSystem), 
		CommandFactory.angleTurnCommand(0.35, 38.33, Direction.RIGHT, driveSystem),
		CommandFactory.driveDistanceCommand(72, Direction.FORWARD, driveSystem)
		);

	private SequentialCommandGroup midScoreAuto = new SequentialCommandGroup(
		CommandFactory.driveDistanceCommand(33.67, Direction.BACKWARD, driveSystem),
		CommandFactory.angleTurnCommand(0.35, 26.69, Direction.RIGHT, driveSystem),
		CommandFactory.driveDistanceCommand(74.953, Direction.BACKWARD, driveSystem),
		CommandFactory.angleTurnCommand(0.35, 26.69, Direction.LEFT, driveSystem),
		CommandFactory.driveDistanceCommand(33, Direction.BACKWARD, driveSystem), new WaitCommand(5),
		CommandFactory.driveDistanceCommand(33.67, Direction.FORWARD, driveSystem),
		CommandFactory.angleTurnCommand(0.35, 26.69, Direction.RIGHT, driveSystem),
		CommandFactory.driveDistanceCommand(74.953, Direction.FORWARD, driveSystem),
		CommandFactory.angleTurnCommand(0.35, 26.69, Direction.LEFT, driveSystem),
		CommandFactory.driveDistanceCommand(52, Direction.FORWARD, driveSystem)
		);

	private SequentialCommandGroup homeTrenchPickupScoreAuto = new SequentialCommandGroup(
		CommandFactory.driveDistanceCommand(158.88, Direction.FORWARD, driveSystem),
		//add parallel command for intake while moving
		//new WaitCommand(10),
		CommandFactory.driveDistanceCommand(218.88, Direction.BACKWARD, driveSystem),
		new WaitCommand(1),
		CommandFactory.angleTurnCommand(0.30, 59.128, Direction.LEFT, driveSystem),
		CommandFactory.driveDistanceCommand(77.95, Direction.BACKWARD, driveSystem),
		CommandFactory.angleTurnCommand(0.30, 59.128, Direction.RIGHT, driveSystem),
		CommandFactory.driveDistanceCommand(1.5, Direction.BACKWARD, driveSystem),
		new WaitCommand(3),
		CommandFactory.driveDistanceCommand(1.5, Direction.FORWARD, driveSystem),
		CommandFactory.angleTurnCommand(0.30, 59.128, Direction.LEFT, driveSystem),
		CommandFactory.driveDistanceCommand(77.95, Direction.FORWARD, driveSystem),
		CommandFactory.angleTurnCommand(0.30, 59.128, Direction.RIGHT, driveSystem),
		CommandFactory.driveDistanceCommand(90, Direction.FORWARD, driveSystem)
	);

	private SequentialCommandGroup awayTrenchPickupScoreAuto = new SequentialCommandGroup(
		CommandFactory.driveDistanceCommand(111.11, Direction.FORWARD, driveSystem),
		CommandFactory.driveDistanceCommand(111.11, Direction.BACKWARD, driveSystem),
		CommandFactory.angleTurnCommand(0.35, 63.97, Direction.RIGHT, driveSystem),
		CommandFactory.driveDistanceCommand(205.11, Direction.BACKWARD, driveSystem),
		CommandFactory.angleTurnCommand(0.35, 63.97, Direction.LEFT, driveSystem),
		CommandFactory.driveDistanceCommand(10.75, Direction.BACKWARD, driveSystem),
		CommandFactory.driveDistanceCommand(140, Direction.FORWARD, driveSystem)
	);

	private SequentialCommandGroup curveAuto = new SequentialCommandGroup(
		CommandFactory.driveCurveCommand(
			//distaces for each side allowing robot to travel a displacement of 120 inches
			(84.8528 + (21.0/2)) * .50 * Math.PI,
			(84.8528 - (21.0/2)) * .50 * Math.PI,
			Direction.FORWARD,
			driveSystem)
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
		intakeIn.whileHeld(CommandFactory.intakeInCommand(intakeSystem));
		intakeIn.whenReleased(CommandFactory.intakeStopCommand(intakeSystem));
		
		JoystickButton intakeOut = new JoystickButton(rightStick, Constants.BUTTON_5);
		intakeOut.whileHeld(CommandFactory.intakeOutCommand(intakeSystem));
		intakeOut.whenReleased(CommandFactory.intakeStopCommand(intakeSystem));
		
		JoystickButton conveyorOut = new JoystickButton(rightStick, Constants.THUMB_BUTTON);
		conveyorOut.whileHeld(CommandFactory.conveyorOutCommand(intakeSystem));
		conveyorOut.whenReleased(CommandFactory.conveyorStopCommand(intakeSystem));

		JoystickButton conveyorIn = new JoystickButton(rightStick, Constants.BUTTON_3);
		conveyorIn.whileHeld(CommandFactory.conveyorInCommand(intakeSystem));
		conveyorIn.whenReleased(CommandFactory.conveyorStopCommand(intakeSystem));
		
		JoystickButton conveyorEmpty = new JoystickButton(rightStick, Constants.BUTTON_4);
		conveyorEmpty.whileHeld(CommandFactory.emptyConveyorCommand(intakeSystem));
		conveyorEmpty.whenReleased(CommandFactory.conveyorStopCommand(intakeSystem));
	}

	public DriveSystem getDriveSystem() {
		return this.driveSystem;
	}

	public IntakeSystem getIntakeSystem() {
		return this.intakeSystem;
	}

	private void configureDefaultCommands() {
		CommandScheduler.getInstance().setDefaultCommand(driveSystem, driveCommand);
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
