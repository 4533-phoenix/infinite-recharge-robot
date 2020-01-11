/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;

public class DriveSystem extends SubsystemBase {
  /**
   * Creates a new DriveSystem.
   */  
  TalonSRX rightMaster;
  TalonSRX rightSlave;
  TalonSRX leftMaster;
  TalonSRX leftSlave;
  private AHRS navX;
  private Port navXPort;
  private double targetL = 0, targetR = 0;
  public static double MAX_VELOCITY = 200;
  public DriveSystem() {
    rightMaster = new TalonSRX(Constants.RIGHT_MASTER_MOTOR);
    rightSlave = new TalonSRX(Constants.RIGHT_SLAVE_MOTOR);
    leftMaster = new TalonSRX(Constants.LEFT_MASTER_MOTOR);
    leftSlave = new TalonSRX(Constants.LEFT_SLAVE_MOTOR);
    navXPort = SPI.Port.kMXP;
    navX = new AHRS(navXPort);
    
    //Allowable margin of error for accurate sensor measurement
    
    
    rightSlave.follow(rightMaster, FollowerType.AuxOutput1);
    leftSlave.follow(leftMaster, FollowerType.AuxOutput1);
    
    rightMaster.setSensorPhase(true);

    leftMaster.setInverted(true);
    leftSlave.setInverted(true);
  }

  public double getAngle() {
    return navX.getAngle();
  }
  
  public void drivePercentOutput(double left,double right) {
    rightMaster.set(ControlMode.PercentOutput,right);
    leftMaster.set(ControlMode.PercentOutput,left);
    //rightSlave.set(ControlMode.PercentOutput,Constants.RIGHT_MASTER_MOTOR);
    //leftSlave.set(ControlMode.PercentOutput,Constants.LEFT_MASTER_MOTOR);
  }

  public void setPIDFValues(double p, double i, double d, double f){
    rightMaster.config_kP(0, p, 100);
    leftMaster.config_kP(0, p, 100);
    rightMaster.config_kI(0, i, 100);
    leftMaster.config_kI(0, i, 100 );
    rightMaster.config_kD(0, d, 100);
    leftMaster.config_kD(0, d, 100 );
    rightMaster.config_kF(0, f, 100);
    leftMaster.config_kF(0, f, 100 );
  }

  public void driveVelocity(double left, double right){
    System.out.println("Left: " + targetL + " " + "Right: " + targetR);
    targetL = left * MAX_VELOCITY * 4096/600.0;
    targetR = right * MAX_VELOCITY * 4096/600.0;
    rightMaster.set(ControlMode.Velocity, targetR);
    leftMaster.set(ControlMode.Velocity, targetL);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
