package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.CommandFactory;
import frc.robot.commands.Direction;
import frc.robot.subsystems.DriveSystem;
import java.util.Map;

public class RobotContainer {
	// Initialize the robots subsystems
	private final DriveSystem driveSystem = new DriveSystem();

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
		CommandFactory.angleTurnCommand(0.35, 38.33, Direction.LEFT, driveSystem));

	private SequentialCommandGroup midScoreGeneratorAuto = new SequentialCommandGroup(
		CommandFactory.driveDistanceCommand(33.67, Direction.BACKWARD, driveSystem),
		CommandFactory.angleTurnCommand(0.35, 26.69, Direction.RIGHT, driveSystem),
		CommandFactory.driveDistanceCommand(74.953, Direction.BACKWARD, driveSystem),
		CommandFactory.angleTurnCommand(0.35, 26.69, Direction.LEFT, driveSystem),
		CommandFactory.driveDistanceCommand(33, Direction.BACKWARD, driveSystem), new WaitCommand(5),
		CommandFactory.driveDistanceCommand(33.67, Direction.FORWARD, driveSystem),
		CommandFactory.angleTurnCommand(0.35, 26.69, Direction.RIGHT, driveSystem),
		CommandFactory.driveDistanceCommand(74.953, Direction.FORWARD, driveSystem),
		CommandFactory.angleTurnCommand(0.35, 26.69, Direction.LEFT, driveSystem),
		CommandFactory.driveDistanceCommand(52, Direction.FORWARD, driveSystem));

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
	private SequentialCommandGroup forwardTurn = new SequentialCommandGroup(
		CommandFactory.driveDistanceCommand(48, Direction.FORWARD, driveSystem)
		//CommandFactory.angleTurnCommand(0.35, 90, Direction.RIGHT, driveSystem)
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
		Map.entry("score_pickup_generator", midScoreGeneratorAuto)
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

	}

	public DriveSystem getDriveSystem() {
		return this.driveSystem;
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
