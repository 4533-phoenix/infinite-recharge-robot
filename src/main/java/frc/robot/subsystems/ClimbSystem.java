/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;

public class ClimbSystem extends SubsystemBase {
  /**
   * Creates a new ClimbSystem.
   */
  private WPI_TalonSRX hookMotor;
  private WPI_TalonSRX climbMotor;
  private final double HOOK_PERCENT_OUTPUT = 0.9;
  private final double CLIMB_PERCENT_OUTPUT = 0.5;

  public ClimbSystem() {
    this.hookMotor = new WPI_TalonSRX(Constants.ELEVATOR_MOTOR);
	this.climbMotor = new WPI_TalonSRX(Constants.WINCH_MOTOR);

	this.hookMotor.setNeutralMode(NeutralMode.Brake);
	this.climbMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void hookUp(){
    this.hookMotor.set(ControlMode.PercentOutput, HOOK_PERCENT_OUTPUT);
  }
  public void hookDown(){
    this.hookMotor.set(ControlMode.PercentOutput, -HOOK_PERCENT_OUTPUT);
  }

  public void hookStop() {
	this.hookMotor.set(ControlMode.PercentOutput, 0);
  }

  public void climb(){
    this.climbMotor.set(ControlMode.PercentOutput, -CLIMB_PERCENT_OUTPUT);
  }

  public void climbStop(){
    this.climbMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}