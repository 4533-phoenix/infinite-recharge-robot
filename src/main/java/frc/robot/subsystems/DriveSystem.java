package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSystem extends SubsystemBase {

	// Drive train motor controllers
	private WPI_TalonSRX rightMaster;
	private WPI_TalonSRX rightSlave;
	private WPI_TalonSRX leftMaster;
	private WPI_TalonSRX leftSlave;

	private DifferentialDrive drive;

	private DifferentialDriveOdometry odometry;

	// Onboard IMU.
	private AHRS navX;

	public static double MAX_VELOCITY = 325;
	private static final double PEAK_OUTPUT = 1.0;

	// Wheel specific constants.
	private static final double TICKS_PER_ROTATION = 4096.0;
	private static final double WHEEL_DIAMETER = 6.0;
	private static final double WHEEL_DIAMETER_M = 0.1524;
	private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
	private static final double WHEEL_CIRCUMFERENCE_M = WHEEL_DIAMETER_M * Math.PI;
	private static final double TICKS_PER_INCH = TICKS_PER_ROTATION / WHEEL_CIRCUMFERENCE;
	private static final double TICKS_PER_METER = TICKS_PER_ROTATION / WHEEL_CIRCUMFERENCE_M;
	private static final double METERS_PER_PULSE = WHEEL_CIRCUMFERENCE_M / TICKS_PER_ROTATION;

	// Velocity PID Gains and Feed Forward values.
	//
	// The following values should be used when driving the robot in "Velocity"
	// mode.
	public static final double VELOCITY_P = 0.000213;
	public static final double VELOCITY_I = 0.0;
	public static final double VELOCITY_D = 0.0;
	public static final double VELOCITY_FEED_FORWARD = 0.243;

	// Position PID Gains and Feed Forward values.
	//
	// The following values should be used when driving the robot in "Position"
	// mode.
	public static final double POSITION_P = 0.055;
	public static final double POSITION_I = 0.0;
	public static final double POSITION_D = 0.0;
	public static final double POSITION_FEED_FORWARD = 0.0;

	// Feed Forward Gains
	//
	// kS - the voltage needed to overcome the motor's static friction (V).
	// kV - the voltage needed to maintain a given constant velocity (V * s/m).
	// kA - the voltage needed to induce a given acceleration (V * s^2/m).
	public static final double FEED_FORWARD_KS = 0.863;
	public static final double FEED_FORWARD_KV = 2.5;
	public static final double FEED_FORWARD_KA = 0.337;

	public static final SimpleMotorFeedforward FEED_FORWARD = new SimpleMotorFeedforward(FEED_FORWARD_KV,
			FEED_FORWARD_KV, FEED_FORWARD_KA);

	// TODO: These values were calculated as part of the robot characterization
	// process. We need to determine whether or not we want to keep them separate
	// from the above PID and FF gains. Another consideration is whether or not
	// we should track the left and right values separately.
	public static final double kPVelocity = 2.38;
	public static final double kDVelocity = 0.0;

	public static final double kPPosition = 0.0;
	public static final double kDPosition = 0.0;

	// Kinematic constants.
	public static final double TRACK_WIDTH = 0.537;
	public static final DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(TRACK_WIDTH);

	public static final double kMaxSpeed = 3;
	public static final double kMaxAcceleration = 3;

	/** Default timeout in milliseconds */
	private static final int DEFAULT_TIMEOUT = 30;

	public DriveSystem() {
		// Initialize all of the drive systems motor controllers.
		this.leftMaster = new WPI_TalonSRX(Constants.LEFT_MASTER_MOTOR);
		this.leftSlave = new WPI_TalonSRX(Constants.LEFT_SLAVE_MOTOR);
		this.rightMaster = new WPI_TalonSRX(Constants.RIGHT_MASTER_MOTOR);
		this.rightSlave = new WPI_TalonSRX(Constants.RIGHT_SLAVE_MOTOR);

		this.leftSlave.follow(leftMaster, FollowerType.AuxOutput1);
		this.rightSlave.follow(rightMaster, FollowerType.AuxOutput1);

		this.leftMaster.setInverted(true);
		this.leftSlave.setInverted(true);

		this.leftMaster.setSensorPhase(false);
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

		this.rightMaster.setNeutralMode(NeutralMode.Brake);
		this.rightSlave.setNeutralMode(NeutralMode.Brake);
		this.leftMaster.setNeutralMode(NeutralMode.Brake);
		this.leftSlave.setNeutralMode(NeutralMode.Brake);

		this.drive = new DifferentialDrive(this.leftMaster, this.rightMaster);

		// Initializing with a 0 angle. This might need to be revisited in the
		// case where we want to start the robot in a different orientation.
		this.odometry = new DifferentialDriveOdometry(new Rotation2d());

		// Initialize the NavX IMU sensor.
		this.navX = new AHRS(SPI.Port.kMXP);
	}

	public void tank(double left, double right) {
	}

	public void setPIDF(double p, double i, double d, double f) {
		// Set PIDF for Left Master controller.
		this.leftMaster.config_kP(0, p, 100);
		this.leftMaster.config_kI(0, i, 100);
		this.leftMaster.config_kD(0, d, 100);
		this.leftMaster.config_kF(0, f, 100);

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

	public double getPosition() {
		return this.leftMaster.getSelectedSensorPosition() / TICKS_PER_INCH;
	}

	public double getLeftDistance() {
		return this.leftMaster.getSelectedSensorPosition() / TICKS_PER_METER;
	}

	public double getRightDistance() {
		return this.rightMaster.getSelectedSensorPosition() / TICKS_PER_METER;
	}

	public double getAngle() {
		return Math.abs(navX.getAngle());
	}

	public void resetAngle() {
		navX.reset();
	}

	public DifferentialDriveWheelSpeeds getWheelSpeeds() {
		// Convert the measured rate of velocity to meters per second.
		//
		// The function 'getSelectedSensorVelocity' returns values that are raw
		// units (pulses or ticks) per 100ms. We need to convert this value into
		// meters per second.
		//
		// First we must convert the raw units in to meters. This is done by
		// multiplying measured value by the value of METERS_PER_PULSE. The result
		// of this will be the measured value in meters per 100ms. Since 1 s =
		// 1000ms, we know that we need to multiply the value by 10, which will give
		// us the meters per second value.
		double left = this.leftMaster.getSelectedSensorVelocity() * METERS_PER_PULSE * 10;
		double right = this.rightMaster.getSelectedSensorVelocity() * METERS_PER_PULSE * 10;
		return new DifferentialDriveWheelSpeeds(left, right);
	}

	public Pose2d getPose() {
		return this.odometry.getPoseMeters();
	}

	public void resetOdometry(Pose2d pose) {
		this.resetPosition();
		this.odometry.resetPosition(pose, Rotation2d.fromDegrees(this.navX.getAngle()));
	  }

	public void tankDriveVolts(double leftVolts, double rightVolts) {
		this.leftMaster.setVoltage(leftVolts);
		this.rightMaster.setVoltage(-rightVolts);
		this.drive.feed();
	  }

	@Override
	public void periodic() {

		// Update the robots odometry.
		this.odometry.update(
			Rotation2d.fromDegrees(this.navX.getAngle()),
			this.getLeftDistance(),
			this.getRightDistance()
		);
	}
}
