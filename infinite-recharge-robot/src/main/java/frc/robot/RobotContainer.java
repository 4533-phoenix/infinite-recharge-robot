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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.IntakeSystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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
  private final IntakeSystem intakeSystem = new IntakeSystem();

  public static Joystick joystick = new Joystick(Constants.DRIVER_CONTROLLER);
  public static JoystickButton intakeInButton = new JoystickButton(joystick, Constants.LEFT_BUMPER);
  public static JoystickButton swingUpButton = new JoystickButton(joystick, Constants.START_BUTTON);
  public static JoystickButton swingDownButton = new JoystickButton(joystick, Constants.BACK_BUTTON);
  //Joystick controller = new Joystick(1);

  private final Command driveCommand = new RunCommand(
    () -> this.driveSystem.tank(
        this.joystick.getRawAxis(1),
        this.joystick.getRawAxis(3)
      ),
    this.driveSystem
    );

  private final Command crossLineCommand = new FunctionalCommand(
    () -> this.driveSystem.resetPosition(),
    () -> this.driveSystem.driveDistance(72),
    (interrupt) -> this.driveSystem.tank(0, 0),
    () -> this.driveSystem.reachedPosition(),
    this.driveSystem
  );
  
  private final Command intakeInCommand = new InstantCommand(
    () -> this.intakeSystem.intakeIn(), 
    this.intakeSystem
  );
  private final Command moveConveyor = new InstantCommand(
    () -> this.intakeSystem.conveyor(),
    this.intakeSystem
  );

  private final Command swingMotorUpCommand = new InstantCommand(
    () -> this.intakeSystem.swingMotorUp(),
    this.intakeSystem
  );

  private final Command swingMotorDownCommand = new InstantCommand(
    () -> this.intakeSystem.swingMotorDown(),
    this.intakeSystem
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
    intakeInButton.whileHeld(intakeInCommand);
    intakeInButton.whileHeld(moveConveyor);
    swingUpButton.whileHeld(swingMotorUpCommand);
    swingDownButton.whileHeld(swingMotorDownCommand);
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
    return this.crossLineCommand;
  }
}
