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
import frc.robot.subsystems.DriveSystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/** 
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handl  in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSystem driveSystem = new DriveSystem();

  Joystick joystick = new Joystick(Constants.DRIVER_CONTROLLER);
  Joystick controller = new Joystick(1);
  JoystickButton aButton = new JoystickButton(joystick, 2);
  JoystickButton bButton = new JoystickButton(joystick, 3);

  private final Command driveCommand = new RunCommand(
    () -> this.driveSystem.tank(
        this.joystick.getRawAxis(1),
        this.joystick.getRawAxis(3) //this.controller.getRawAxis(1)
      ),  
    this.driveSystem
    );

    // private final Command autoCommand = new StartEndCommand(
    //   () -> this.driveSystem.drivePosition(driveSystem.getEncoderValues(-10)),
    //   this.driveSystem
    // );
    private final Command autoCommand = new StartEndCommand(
      // this is the onInit
      () -> this.driveSystem.drivePosition(driveSystem.getEncoderValues(-155)),
      // this is the onEnd
      () -> this.driveSystem.tank(0,0),
      // this is the Requirements
      driveSystem);
  
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
    aButton.whenPressed(autoCommand);
    bButton.whenPressed(driveCommand);
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
