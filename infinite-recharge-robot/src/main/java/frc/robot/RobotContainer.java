/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The  code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.CommandFactory;
import frc.robot.subsystems.DriveSystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import javax.naming.directory.DirContext;
import javax.xml.crypto.dsig.keyinfo.RetrievalMethod;
import frc.robot.commands.Direction;

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSystem driveSystem = new DriveSystem();
  public static Joystick joystick = new Joystick(Constants.DRIVER_CONTROLLER);
  Joystick controller = new Joystick(1);
  public static JoystickButton intakeButton =
		  new JoystickButton(joystick, Constants.LEFT_BUMPER);

  private final Command driveCommand = new RunCommand(
    () -> this.driveSystem.tank(
        this.joystick.getRawAxis(1),
        this.controller.getRawAxis(1)
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
    double pValue = 0.15;
    double iValue = 0.0;
    double dValue = 2.5;
    double fValue = 0.243;
    configureButtonBindings();
    configureDefaultCommands();
    this.driveSystem.setPIDF(pValue, iValue, dValue, fValue);
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
    return  this.squareAuto;
  }
}
