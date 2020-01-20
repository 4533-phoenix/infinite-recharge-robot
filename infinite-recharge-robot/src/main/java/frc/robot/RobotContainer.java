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
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ClimbSystem;
import frc.robot.subsystems.DriveSystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;

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
  private final ClimbSystem climbSystem = new ClimbSystem();
  public static Joystick joystick = new Joystick(Constants.DRIVER_CONTROLLER);
  JoystickButton hookButton = new JoystickButton(joystick, Constants.X_BUTTON);
  JoystickButton climbButton = new JoystickButton(joystick, Constants.Y_BUTTON);
  //Joystick controller = new Joystick(1);

  private final Command driveCommand = new RunCommand(
    () -> this.driveSystem.tank(
        this.joystick.getRawAxis(1),
        this.joystick.getRawAxis(3) //this.controller.getRawAxis(1)
      ),  
    this.driveSystem
    );

  private final Command climbCommand = new InstantCommand(() -> climbSystem.climb(), this.climbSystem);
  
  
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
  /*public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }*/
}
