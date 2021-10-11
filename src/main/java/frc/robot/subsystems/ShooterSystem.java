package frc.robot.subsystems;

import java.util.ResourceBundle.Control;
import static java.lang.Math.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.ConnectionInfo;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Direction;

import com.ctre.phoenix.motorcontrol.NeutralMode;

public class ShooterSystem extends SubsystemBase {
	
	private double flywheelMotorPercent = 0;

	private final double TURRET_WHEEL_MOTOR_PERCENT = 0.5;

	private final double TURRET_SWIVEL_MOTOR_PERCENT = 0.1;

	private WPI_TalonFX flywheelMotorRight;
	private WPI_TalonFX flywheelMotorLeft;

	private WPI_VictorSPX turretWheelMotor;

	private WPI_TalonSRX turretSwivelMotor;

	private NetworkTable inst;

	private double targetOffsetAngle_Horizontal;
	private double targetOffsetAngle_Vertical;
	private double targetArea;
	private double targetSkew;
	private double[] camtran = {0};

	private double cameraHeight = 1.75; //height is in feet
	private double goalHeight = 8.25; // height is in feet
	private double cameraMountingAngle = (Math.PI / 180) * 26.5;
	private double cameraTargetAngle = 0;
	private double launchAngle = (Math.PI / 180) * 26.5;
	private double verticalPosition = goalHeight - cameraHeight;
	private double horizontalPosition = 0;
	private double initialVelocity = 0;
	private final double GRAVITY_ACCELERATION =  32.17;
	private final double FLYWHEEL_RADIUS = 1.0 / 3.0 / 2.0;
	private final double MAX_FLYWHEEL_RPM = 3220.0;

	private double startFlywheelRotations = 0;
	private double currFlywheelRotations = 0;
	private double flywheelRPM = 0;
	private double idealFlywheelRPM = 0;
	private double elapsedTime = 0;

	public ShooterSystem() {
		this.flywheelMotorRight = new WPI_TalonFX(Constants.FLYWHEEL_MOTOR_RIGHT);
		this.flywheelMotorLeft = new WPI_TalonFX(Constants.FLYWHEEL_MOTOR_LEFT);

		this.flywheelMotorRight.setNeutralMode(NeutralMode.Brake);
		this.flywheelMotorLeft.setNeutralMode(NeutralMode.Brake);

		this.turretWheelMotor = new WPI_VictorSPX(Constants.TURRET_WHEEL_MOTOR);

		this.turretWheelMotor.setInverted(true);

		this.turretWheelMotor.setNeutralMode(NeutralMode.Brake);

		this.turretSwivelMotor = new WPI_TalonSRX(Constants.TURRET_SWIVEL_MOTOR);

		this.turretSwivelMotor.setNeutralMode(NeutralMode.Brake);

		this.turretSwivelMotor.setInverted(true);

		this.flywheelMotorLeft.setInverted(true);

		this.inst = NetworkTableInstance.getDefault().getTable("limelight");
		
		// ConnectionInfo[] arr = this.inst.getConnections();

		// for(ConnectionInfo curr : arr) {
		// 	System.out.println(curr.remote_id + " " + curr.remote_ip);
		// }

		startFlywheelRotations = flywheelMotorRight.getSelectedSensorPosition();
	}

	public void flywheelOut() {
		this.flywheelMotorRight.set(ControlMode.PercentOutput, flywheelMotorPercent);
		this.flywheelMotorLeft.set(ControlMode.PercentOutput, flywheelMotorPercent);
	}

	public void flywheelIn() {
		this.flywheelMotorRight.set(ControlMode.PercentOutput, -flywheelMotorPercent);
		this.flywheelMotorLeft.set(ControlMode.PercentOutput, -flywheelMotorPercent);
	}

	public void flywheelStop() {
		this.flywheelMotorRight.set(ControlMode.PercentOutput, 0);
		this.flywheelMotorLeft.set(ControlMode.PercentOutput, 0);
	}
	
	public void flywheelAndIntakeOut() {
		this.flywheelMotorRight.set(ControlMode.PercentOutput, flywheelMotorPercent);
		this.flywheelMotorLeft.set(ControlMode.PercentOutput, flywheelMotorPercent);
		this.turretWheelMotor.set(ControlMode.PercentOutput, TURRET_WHEEL_MOTOR_PERCENT);
	}

	public void flywheelAndIntakeStop() {
		this.flywheelMotorRight.set(ControlMode.PercentOutput, 0);
		this.flywheelMotorLeft.set(ControlMode.PercentOutput, 0);
		this.turretWheelMotor.set(ControlMode.PercentOutput, 0);
	}

	public void flywheelAndIntakeResetPosition() {
		this.flywheelMotorRight.setSelectedSensorPosition(0);
		this.flywheelMotorLeft.setSelectedSensorPosition(0);
		this.turretWheelMotor.setSelectedSensorPosition(0);
	}

	public boolean flywheelReachedPosition(int balls) {
		double targetPosition = (balls * 30) * DriveSystem.TICKS_PER_ROTATION;
		return flywheelMotorRight.getSelectedSensorPosition() >= targetPosition;
	}

	public void word() {
		System.out.println("test2");
	}

	public void turretWheelIn() {
		// since both flywheel motors should be at the same position, we only need to check one flywheel motor's position
		if (flywheelRPM >= 0.95 * idealFlywheelRPM) {
			this.turretWheelMotor.set(ControlMode.PercentOutput, TURRET_WHEEL_MOTOR_PERCENT);
		}
		else {
			this.turretWheelMotor.set(ControlMode.PercentOutput, 0);
		}
	}

	public void turretWheelOut() {
		this.turretWheelMotor.set(ControlMode.PercentOutput, -TURRET_WHEEL_MOTOR_PERCENT);
	}

	public void turretWheelStop() {
		this.turretWheelMotor.set(ControlMode.PercentOutput, 0);
	}

	public void turretSwivelLeft() {
		this.turretSwivelMotor.set(ControlMode.PercentOutput, TURRET_SWIVEL_MOTOR_PERCENT);
	}

	public void turretSwivelRight() {
		this.turretSwivelMotor.set(ControlMode.PercentOutput, -TURRET_SWIVEL_MOTOR_PERCENT);
	}

	public void turretSwivelStop() {
		this.turretSwivelMotor.set(ControlMode.PercentOutput, 0);
	}

	public void autoTurretSwivel() {
		// System.out.println("test");
		if (targetOffsetAngle_Horizontal > 1) {
			this.turretSwivelMotor.set(ControlMode.PercentOutput, -TURRET_SWIVEL_MOTOR_PERCENT);
		} 
		else if (targetOffsetAngle_Horizontal < -1) {
			this.turretSwivelMotor.set(ControlMode.PercentOutput, TURRET_SWIVEL_MOTOR_PERCENT);
		}
	}

	public boolean turretReachedPosition() {
		return targetOffsetAngle_Horizontal > -1 && targetOffsetAngle_Horizontal < 1;
	}

	@Override
	public void periodic() {
		targetOffsetAngle_Horizontal = inst.getEntry("tx").getDouble(0);
		targetOffsetAngle_Vertical = inst.getEntry("ty").getDouble(0);
		targetArea = inst.getEntry("ta").getDouble(0);
		targetSkew = inst.getEntry("ts").getDouble(0);
		camtran = inst.getEntry("camtran").getDoubleArray(camtran);

		cameraTargetAngle = (Math.PI / 180) * targetOffsetAngle_Vertical;
		
		horizontalPosition = verticalPosition / tan((cameraMountingAngle + cameraTargetAngle)); // horizontalPosition is distance from the goal

		initialVelocity = sqrt(abs((GRAVITY_ACCELERATION * pow(horizontalPosition, 2)) / ((2 * pow(cos(launchAngle), 2)) * ((-1 * verticalPosition) + (horizontalPosition * tan(launchAngle))))));

		idealFlywheelRPM = initialVelocity / 6;

		flywheelMotorPercent = idealFlywheelRPM * 1000 / MAX_FLYWHEEL_RPM;

		// periodic runs every 20 ms
		elapsedTime += 20;
		
		if (elapsedTime == 100) {
			elapsedTime = 0;
			currFlywheelRotations = flywheelMotorRight.getSelectedSensorPosition() / 4096.0; // 4096.0 is ticks per rotation
			flywheelRPM = (currFlywheelRotations - startFlywheelRotations) * 600; // multiplying by 600 to get to minutes
			startFlywheelRotations = currFlywheelRotations;
		}

		// System.out.println("Distance in Feet: " + horizontalPosition);

		// System.out.println("Initial Velocity: " + initialVelocity);

		System.out.println("Ideal Flywheel RPM: " + idealFlywheelRPM);
		
		System.out.println("Flywheel Motor Percent: " + flywheelMotorPercent);
	}
}
