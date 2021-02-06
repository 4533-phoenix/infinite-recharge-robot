package frc.robot;

import java.util.List;
import java.util.Map;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.CommandFactory;
import frc.robot.subsystems.DriveSystem;

public class RobotContainer {
	// Initialize the driver controls
	private Joystick leftStick = new Joystick(Constants.DRIVER_JOYSTICK_LEFT);
	private Joystick rightStick = new Joystick(Constants.DRIVER_JOYSTICK_RIGHT);

	// Initialize the drive command
	private final Command defaultDriveCommand = new RunCommand(
		() -> Robot.drive.tank(
			this.leftStick.getRawAxis(1),
			this.rightStick.getRawAxis(1)
		),
		Robot.drive
	);


	//creates a hashMap
	private Map<String, Command> commands = Map.ofEntries();

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
		// Create a voltage constraint to ensure we don't accelerate too fast
		var autoVoltageConstraint =
			new DifferentialDriveVoltageConstraint(
				DriveSystem.FEED_FORWARD,
				DriveSystem.KINEMATICS,
				10
			);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(3,3)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveSystem.KINEMATICS)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(270)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(1, 0),
            new Translation2d(2, 0)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        config
    );

    RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        Robot.drive::getPose,
        new RamseteController(2, 0.7),
        DriveSystem.FEED_FORWARD,
        DriveSystem.KINEMATICS,
        Robot.drive::getWheelSpeeds,
        new PIDController(DriveSystem.kPVelocity, 0, 0),
        new PIDController(DriveSystem.kPVelocity, 0, 0),
        // RamseteCommand passes volts to the callback
        Robot.drive::tankDriveVolts,
        Robot.drive
    );

    // Reset odometry to the starting pose of the trajectory.
    Robot.drive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> Robot.drive.tankDriveVolts(0, 0));
	}
}
