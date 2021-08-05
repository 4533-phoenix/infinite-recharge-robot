package frc.robot.subsystems;

import java.util.ResourceBundle.Control;

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
	
	private final double FLYWHEEL_MOTOR_PERCENT = 0.75;

	private final double ELEVATOR_MOTOR_PERCENT = 0.5;

	private final double TURRET_WHEEL_MOTOR_PERCENT = 0.5;

	private final double TURRET_SWIVEL_MOTOR_PERCENT = 0.1;

	private WPI_TalonFX flywheelMotorRight;
	private WPI_TalonFX flywheelMotorLeft;

	private WPI_VictorSPX turretWheelMotor;

	private WPI_TalonSRX turretSwivelMotor;

	private WPI_TalonSRX elevatorMotor;

	private NetworkTable inst;

	private double targetOffsetAngle_Horizontal;

	private double startFlywheelRotations = flywheelMotorRight.getSelectedSensorPosition() / 4096.0;
	private double currFlywheelRotations;
	private double flywheelRPM;
	private double elapsedTime;

	public ShooterSystem() {

		this.flywheelMotorRight = new WPI_TalonFX(Constants.FLYWHEEL_MOTOR_RIGHT);
		this.flywheelMotorLeft = new WPI_TalonFX(Constants.FLYWHEEL_MOTOR_LEFT);

		this.flywheelMotorRight.setNeutralMode(NeutralMode.Brake);
		this.flywheelMotorLeft.setNeutralMode(NeutralMode.Brake);

		//this.elevatorMotor = new WPI_TalonSRX(Constants.ELEVATOR_MOTOR);

		//this.elevatorMotor.setNeutralMode(NeutralMode.Brake);

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
	}

	public void flywheelOut() {
		this.flywheelMotorRight.set(ControlMode.PercentOutput, FLYWHEEL_MOTOR_PERCENT);
		this.flywheelMotorLeft.set(ControlMode.PercentOutput, FLYWHEEL_MOTOR_PERCENT);
	}

	public void flywheelIn() {
		this.flywheelMotorRight.set(ControlMode.PercentOutput, -FLYWHEEL_MOTOR_PERCENT);
		this.flywheelMotorLeft.set(ControlMode.PercentOutput, -FLYWHEEL_MOTOR_PERCENT);
	}

	public void flywheelStop() {
		this.flywheelMotorRight.set(ControlMode.PercentOutput, 0);
		this.flywheelMotorLeft.set(ControlMode.PercentOutput, 0);
	}
	
	public void flywheelAndIntakeOut() {
		this.flywheelMotorRight.set(ControlMode.PercentOutput, FLYWHEEL_MOTOR_PERCENT);
		this.flywheelMotorLeft.set(ControlMode.PercentOutput, FLYWHEEL_MOTOR_PERCENT);
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
		double targetPosition = (balls * 10 + 5) * DriveSystem.TICKS_PER_ROTATION;
		return flywheelMotorRight.getSelectedSensorPosition() >= targetPosition;
	}

	public void word() {
		System.out.println("test2");
	}

	public void turretWheelIn() {
		// since both flywheel motors should be at the same position, we only need to check one flywheel motor's position
		// if (flywheelMotorRight.getSelectedSensorPosition() >= 4096 * 3) {
		// 	this.turretWheelMotor.set(ControlMode.PercentOutput, TURRET_WHEEL_MOTOR_PERCENT);
		// }
		// else {
		// 	this.turretWheelMotor.set(ControlMode.PercentOutput, 0);
		// }

		this.turretWheelMotor.set(ControlMode.PercentOutput, TURRET_WHEEL_MOTOR_PERCENT);
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
		if (targetOffsetAngle_Horizontal > 2) {
			while (targetOffsetAngle_Horizontal > 2) {
				this.turretSwivelMotor.set(ControlMode.PercentOutput, -TURRET_SWIVEL_MOTOR_PERCENT);
			}
		} else if (targetOffsetAngle_Horizontal < -2) {
			while (targetOffsetAngle_Horizontal < -2) {
				this.turretSwivelMotor.set(ControlMode.PercentOutput, TURRET_SWIVEL_MOTOR_PERCENT);
			}
		}

		this.turretSwivelStop();
	}

	public boolean turretReachedPosition() {
		return targetOffsetAngle_Horizontal < -2 || targetOffsetAngle_Horizontal > 2;
	}

	@Override
	public void periodic() {
		targetOffsetAngle_Horizontal = inst.getEntry("tx").getDouble(0);
		double targetOffsetAngle_Vertical = inst.getEntry("ty").getDouble(0);
		double targetArea = inst.getEntry("ta").getDouble(0);
		double targetSkew = inst.getEntry("ts").getDouble(0);

		// periodic runs every 20 ms
		elapsedTime += 20;
		
		if (elapsedTime == 100) {
			elapsedTime = 0;
			currFlywheelRotations = flywheelMotorRight.getSelectedSensorPosition() / 4096.0;
			flywheelRPM = (currFlywheelRotations - startFlywheelRotations) * 600;
			startFlywheelRotations = flywheelMotorRight.getSelectedSensorPosition() / 4096.0;
		}

		if (flywheelRPM <= 0.0) {
			flywheelRPM = 0.0;
		}

		System.out.println("Flywheel RPM: " + flywheelRPM);

		// double test = flywheelMotorRight.get();

		

		// System.out.println(targetOffsetAngle_Horizontal);
	}
}
