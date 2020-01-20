/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSystem extends SubsystemBase {
  /**
   * Creates a new ClimbSystem.
   */
  private VictorSPX hookMotor;
  private VictorSPX climbMotor;
  private DigitalInput hookUpperLimit;
  private DigitalInput hookLowerLimit;
  private final double HOOK_PERCENT_OUTPUT = 0.5;
  private final double CLIMB_PERCENT_OUTPUT = 0.5;

  public ClimbSystem() {
    this.hookMotor = new VictorSPX(Constants.HOOK_MOTOR);
    this.climbMotor = new VictorSPX(Constants.CLIMB_MOTOR);
    this.hookUpperLimit = new DigitalInput(Constants.HOOK_UPPER_LIMIT);
    this.hookLowerLimit = new DigitalInput(Constants.HOOK_LOWER_LIMIT);
  }

  public void hookUp(){
    if(!hookUpperLimit.get()){
      this.hookMotor.set(ControlMode.PercentOutput, HOOK_PERCENT_OUTPUT);
    } else {
      this.hookMotor.set(ControlMode.PercentOutput, 0);
    }
  }
  public void hookDown(){
    if(!hookLowerLimit.get()){
      this.hookMotor.set(ControlMode.PercentOutput, -HOOK_PERCENT_OUTPUT);
    } else {
      this.hookMotor.set(ControlMode.PercentOutput, 0);
    }
  }

  public void climb(){
    this.climbMotor.set(ControlMode.PercentOutput, CLIMB_PERCENT_OUTPUT);
  }
  public void climbStop(){
    this.climbMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
