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

  private static final double TICKS_PER_ROTATION = 4096.0;
  private static final double WHEEL_DIAMETER = 8.0;
  private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
  private static final double TICKS_PER_INCH = TICKS_PER_ROTATION / WHEEL_CIRCUMFERENCE;

  /** Default timeout in milliseconds */
  private static final int DEFAULT_TIMEOUT = 30;

  private static double targetPosition = 0;

  public DriveSystem() {
    // Initialize all of the drive systems motor controllers.
    this.leftMaster = new TalonSRX(Constants.LEFT_MASTER_MOTOR);
    this.leftSlave = new TalonSRX(Constants.LEFT_SLAVE_MOTOR);
    this.rightMaster = new TalonSRX(Constants.RIGHT_MASTER_MOTOR);
    this.rightSlave = new TalonSRX(Constants.RIGHT_SLAVE_MOTOR);


    this.leftSlave.follow(leftMaster, FollowerType.AuxOutput1);
    this.rightSlave.follow(rightMaster, FollowerType.AuxOutput1);

    this.leftMaster.setInverted(true);
    this.leftSlave.setInverted(true);

    this.leftMaster.setSensorPhase(true);
    this.rightMaster.setSensorPhase(true);

    this.leftMaster.configPeakOutputForward(PEAK_OUTPUT);
    this.leftMaster.configPeakOutputReverse(-PEAK_OUTPUT);
    this.leftSlave.configPeakOutputForward(PEAK_OUTPUT);
    this.leftSlave.configPeakOutputReverse(-PEAK_OUTPUT);

    this.rightMaster.configPeakOutputForward(PEAK_OUTPUT);
    this.rightMaster.configPeakOutputReverse(-PEAK_OUTPUT);
    this.rightSlave.configPeakOutputForward(PEAK_OUTPUT);
    this.rightSlave.configPeakOutputReverse(-PEAK_OUTPUT);

    this.leftMaster.configNominalOutputForward(0, DEFAULT_TIMEOUT);
    this.leftMaster.configNominalOutputReverse(0, DEFAULT_TIMEOUT);
    this.leftSlave.configNominalOutputForward(0, DEFAULT_TIMEOUT);
    this.leftSlave.configNominalOutputReverse(0, DEFAULT_TIMEOUT);

    this.rightMaster.configNominalOutputForward(0, 30);
    this.rightMaster.configNominalOutputReverse(0, 30);
    this.rightSlave.configNominalOutputForward(0, 30);
    this.rightSlave.configNominalOutputReverse(0, 30);

    this.leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, DEFAULT_TIMEOUT);
    this.rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, DEFAULT_TIMEOUT);

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

  public void resetPosition() {
    this.rightMaster.setSelectedSensorPosition(0);
    this.leftMaster.setSelectedSensorPosition(0);
  }

  public boolean reachedPosition() {
    double leftPos = this.leftMaster.getSelectedSensorPosition();
    double rightPos = this.rightMaster.getSelectedSensorPosition();
    System.out.printf("Target: %f - Position: (%f, %f)\n", targetPosition, leftPos, rightPos);

    return (leftPos >= targetPosition) && (rightPos >= targetPosition);
  }

  public void driveDistance(double inches) {
    targetPosition = inches * TICKS_PER_INCH;
    this.leftMaster.set(ControlMode.Position, targetPosition);
    this.rightMaster.set(ControlMode.Position, targetPosition);
  }

  public void tank(double left, double right) {
    double targetLeft = left * MAX_VELOCITY * 4096/600.0;
    double targetRight = right * MAX_VELOCITY * 4096/600.0;

    //System.out.println("Left: "+left + " Right: " + right);

    this.leftMaster.set(ControlMode.Velocity, targetLeft);
    this.rightMaster.set(ControlMode.Velocity, targetRight);
  }

  public void percent(double left, double right) {
    rightMaster.set(ControlMode.PercentOutput, right);
    leftMaster.set(ControlMode.PercentOutput, left);
    rightSlave.set(ControlMode.Follower, Constants.RIGHT_MASTER_MOTOR);
    leftSlave.set(ControlMode.Follower, Constants.LEFT_MASTER_MOTOR);
  }

  public double getAngle(){
    return Math.abs(navX.getAngle());
  }

  public void resetAngle(){
    navX.reset();
  }

  public void turn(double speed, String direction){
    int degree = 90;
    if (degree - 25 <= this.getAngle()) {
      this.tank(speed*.1, -speed*.1);
      System.out.println("Angle A: " + this.getAngle());
    } else {
      this.tank(speed, -speed);
      System.out.println("Angle B: " + this.getAngle());
    }
  }

  @Override
  public void periodic() {
    //System.out.println("Angle: " + this.getAngle());
  }
}
