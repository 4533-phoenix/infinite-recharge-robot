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
  public DriveSystem() {
    rightMaster = new TalonSRX(Constants.RIGHT_MASTER_MOTOR);
    rightSlave = new TalonSRX(Constants.RIGHT_SLAVE_MOTOR);
    leftMaster = new TalonSRX(Constants.LEFT_MASTER_MOTOR);
    leftSlave = new TalonSRX(Constants.LEFT_SLAVE_MOTOR);
    navXPort = SPI.Port.kMXP;
    navX = new AHRS(navXPort);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,100);
    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,100);
    //Allowable margin of error for accurate sensor measurement
    rightMaster.configAllowableClosedloopError(0,50,100);
    leftMaster.configAllowableClosedloopError(0,50,100);
    rightSlave.follow(rightMaster);
    leftSlave.follow(leftMaster);
  }
  public double getAngle() {
    return navX.getAngle();
  }
  public void drivePercentOutput(double left,double right) {
    rightMaster.set(ControlMode.PercentOutput,right);
    leftMaster.set(ControlMode.PercentOutput,left);
    rightSlave.set(ControlMode.PercentOutput,Constants.RIGHT_MASTER_MOTOR);
    leftSlave.set(ControlMode.PercentOutput,Constants.LEFT_MASTER_MOTOR);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
