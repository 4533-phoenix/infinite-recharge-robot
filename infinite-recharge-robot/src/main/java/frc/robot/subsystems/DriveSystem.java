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
  public TalonSRX rightMaster;
  private TalonSRX rightSlave;
  public TalonSRX leftMaster;
  private TalonSRX leftSlave;
  private AHRS navX;
  private Port navXPort;

  public static double MAX_VELOCITY = 450;
  private static final double PEAK_OUTPUT = 0.5;

  public DriveSystem() {
    // Initialize all of the drive systems motor controllers.
    this.leftMaster = new TalonSRX(Constants.LEFT_MASTER_MOTOR);
    this.leftSlave = new TalonSRX(Constants.LEFT_SLAVE_MOTOR);
    this.rightMaster = new TalonSRX(Constants.RIGHT_MASTER_MOTOR);
    this.rightSlave = new TalonSRX(Constants.RIGHT_SLAVE_MOTOR);

    
    this.leftSlave.follow(leftMaster, FollowerType.AuxOutput1);
    this.rightSlave.follow(rightMaster, FollowerType.AuxOutput1);

    this.leftMaster.setInverted(true);
    leftSlave.setInverted(true);
    this.leftMaster.setSensorPhase(true);
    // this.rightMaster.setSensorPhase(true);
    this.rightMaster.setSensorPhase(true);

    this.leftMaster.configPeakOutputForward(PEAK_OUTPUT);
    this.leftMaster.configPeakOutputReverse(-PEAK_OUTPUT);
    this.leftSlave.configPeakOutputForward(PEAK_OUTPUT);
    this.leftSlave.configPeakOutputReverse(-PEAK_OUTPUT);

    this.rightMaster.configPeakOutputForward(PEAK_OUTPUT);
    this.rightMaster.configPeakOutputReverse(-PEAK_OUTPUT);
    this.rightSlave.configPeakOutputForward(PEAK_OUTPUT);
    this.rightSlave.configPeakOutputReverse(-PEAK_OUTPUT);

    this.leftMaster.configNominalOutputForward(0, 30);
    this.leftMaster.configNominalOutputReverse(0, 30);
    this.leftSlave.configNominalOutputForward(0, 30);
    this.leftSlave.configNominalOutputReverse(0, 30);
    
    this.rightMaster.configNominalOutputForward(0, 30);
    this.rightMaster.configNominalOutputReverse(0, 30);
    this.rightSlave.configNominalOutputForward(0, 30);
    this.rightSlave.configNominalOutputReverse(0, 30);
    
    this.leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
    this.rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);

    // Initialize the NavX IMU sensor.
    this.navXPort = SPI.Port.kMXP;
    this.navX = new AHRS(navXPort);
  }

  public void setPIDF(double p, double i, double d, double f){
    // Set PIDF for Left Master controller.
    this.leftMaster.config_kP(0, p, 100);
    this.leftMaster.config_kI(0, i, 100 );
    this.leftMaster.config_kD(0, d, 100 );
    this.leftMaster.config_kF(0, f, 100 );

    // Set PIDF for Right Master controller.
    this.rightMaster.config_kP(0, p, 100);
    this.rightMaster.config_kI(0, i, 100);
    this.rightMaster.config_kD(0, d, 100);
    this.rightMaster.config_kF(0, f, 100);
  }

  public void tank(double left, double right) {
    double targetLeft = left * MAX_VELOCITY * 4096/600.0;
    double targetRight = right * MAX_VELOCITY * 4096/600.0;

    System.out.println("Left: "+left + " Right: " + right);

    this.leftMaster.set(ControlMode.Velocity, targetLeft);
    this.rightMaster.set(ControlMode.Velocity, targetRight);
  }

  public void drivePosition(double position) {
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
    leftMaster.set(ControlMode.Position, position);
    rightMaster.set(ControlMode.Position, position);
  }

  public void setPosition(int position) {
    leftMaster.setSelectedSensorPosition(position, 0, 100);
    rightMaster.setSelectedSensorPosition(position, 0, 100);
  }

  public double getLeftPosition() {
    return leftMaster.getSelectedSensorPosition(0);
  }
  public double getRightPosition() {
    return rightMaster.getSelectedSensorPosition(0);
  }
  public double getEncoderValues(double distanceInInches) {
    final double twelveFootTicks = 12098.7213;
    final double baseDistanceInches = 144.0;
    final double distanceEncoderValue = (distanceInInches * twelveFootTicks) / baseDistanceInches;
    // double wheelSize = 6.0; // enter wheel size in inches decimal form
    // double encoderTicks = 4096.0; // enter number of encoder ticks in decimal form
    // final double CIRCUMFERENCE = wheelSize * Math.PI;
    // final double UNITS_PER_INCH = encoderTicks / CIRCUMFERENCE;
    // final double distance = distanceInInches * UNITS_PER_INCH;
    return distanceEncoderValue;
  }

  @Override
  public void periodic() {
    // System.out.printf("Left: %d - Right: %d\n",
    //   this.leftMaster.getSelectedSensorVelocity(0),
    //   this.rightMaster.getSelectedSensorVelocity(0)
    // );
  }
}
