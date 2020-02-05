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
        this.rightStick.getRawAxis(1)
      ),
    this.driveSystem
  );

  private SequentialCommandGroup squareAuto = new SequentialCommandGroup(
    CommandFactory.driveDistanceCommand(24, Direction.FORWARD, driveSystem),
    CommandFactory.angleTurnCommand(.25, 90, Direction.LEFT, driveSystem),
    CommandFactory.driveDistanceCommand(24, Direction.FORWARD, driveSystem),
    CommandFactory.angleTurnCommand(.25, 90, Direction.LEFT, driveSystem),
    CommandFactory.driveDistanceCommand(24, Direction.FORWARD, driveSystem),
    CommandFactory.angleTurnCommand(.25, 90, Direction.LEFT, driveSystem),
    CommandFactory.driveDistanceCommand(24, Direction.FORWARD, driveSystem),
    CommandFactory.angleTurnCommand(.25, 90, Direction.LEFT, driveSystem)
  );

  private SequentialCommandGroup goalAuto = new SequentialCommandGroup(
    CommandFactory.driveDistanceCommand(101, Direction.BACKWARD, driveSystem),
    new WaitCommand(5),
    CommandFactory.driveDistanceCommand(120, Direction.FORWARD, driveSystem),
    CommandFactory.angleTurnCommand(0.35, 38.33, Direction.LEFT, driveSystem)
  );

  private SequentialCommandGroup midAuto = new SequentialCommandGroup(
    CommandFactory.driveDistanceCommand(33.67, Direction.BACKWARD, driveSystem),
    CommandFactory.angleTurnCommand(0.35, 26.69, Direction.RIGHT, driveSystem),
    CommandFactory.driveDistanceCommand(74.953, Direction.BACKWARD, driveSystem),
    CommandFactory.angleTurnCommand(0.35, 26.69, Direction.LEFT, driveSystem),
    CommandFactory.driveDistanceCommand(33, Direction.BACKWARD, driveSystem),
    new WaitCommand(5),
    CommandFactory.driveDistanceCommand(33.67, Direction.FORWARD, driveSystem),
    CommandFactory.angleTurnCommand(0.35, 26.69, Direction.RIGHT, driveSystem),
    CommandFactory.driveDistanceCommand(74.953, Direction.FORWARD, driveSystem),
    CommandFactory.angleTurnCommand(0.35, 26.69, Direction.LEFT, driveSystem),
    CommandFactory.driveDistanceCommand(52, Direction.FORWARD, driveSystem)
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

  private void configureDefaultCommands(){
    CommandScheduler.getInstance().setDefaultCommand(driveSystem, driveCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return CommandFactory.getTrajectoryCommand(driveSystem);
  }
}
