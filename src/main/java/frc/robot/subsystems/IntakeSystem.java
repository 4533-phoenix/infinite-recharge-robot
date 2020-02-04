/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class IntakeSystem extends SubsystemBase {
  /**
   * Creates a new IntakeSystem.
   */
  private VictorSPX intakeMotor;
  private VictorSPX conveyorMotor;
  private VictorSPX swingMotor;

  private AnalogInput poten;

  private DigitalInput powerCell1; 
  private DigitalInput powerCell2; 
  private DigitalInput powerCell3; 
  private DigitalInput powerCell4; 
  private DigitalInput powerCell5; 

  private final double INTAKE_MOTOR_PERCENT = 0.5;
  private final double CONVEYOR_MOTOR_PERCENT = 0.5;
  private final double SWING_MOTOR_PERCENT = 0.5;
  

  public IntakeSystem() {
    intakeMotor = new VictorSPX(Constants.INTAKE_MOTOR);
    conveyorMotor = new VictorSPX(Constants.CONVEYOR_MOTOR);
    swingMotor = new VictorSPX(Constants.SWING_MOTOR);
    poten = new AnalogInput(Constants.POTEN);
    powerCell1 = new DigitalInput(Constants.POWER_CELL_1);     
    powerCell2 = new DigitalInput(Constants.POWER_CELL_2);     
    powerCell3 = new DigitalInput(Constants.POWER_CELL_3);     
    powerCell4 = new DigitalInput(Constants.POWER_CELL_4); 
    powerCell5 = new DigitalInput(Constants.POWER_CELL_5);     
  }

  public void intakeIn(){
    this.intakeMotor.set(ControlMode.PercentOutput, INTAKE_MOTOR_PERCENT);
  }
  public void conveyor(){
    this.conveyorMotor.set(ControlMode.PercentOutput, CONVEYOR_MOTOR_PERCENT);
  }
  public void swingMotorUp(){
    this.swingMotor.set(ControlMode.PercentOutput, SWING_MOTOR_PERCENT);
  }
  public void swingMotorDown(){
    this.swingMotor.set(ControlMode.PercentOutput, -SWING_MOTOR_PERCENT);
  }
  public double position(){
    return poten.getValue();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
