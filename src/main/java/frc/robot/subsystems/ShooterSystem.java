package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Direction;

import com.ctre.phoenix.motorcontrol.NeutralMode;

public class ShooterSystem {
	
	private final double FLYWHEEL_MOTOR_PERCENT = 1;

	private final double ELEVATOR_MOTOR_PERCENT = 0.5;

	private final double TURRET_WHEEL_MOTOR_PERCENT = 0.5;

	private final double TURRET_SWIVEL_MOTOR_PERCENT = 0.5;

	private WPI_TalonSRX flyWheelMotor;

	private WPI_TalonSRX turretWheelMotor;

	private WPI_TalonSRX turretSwivelMotor;

	private WPI_TalonSRX elevatorMotor;

	public ShooterSystem() {

		this.flyWheelMotor = new WPI_TalonSRX(Constants.FLYWHEEL_MOTOR);

		this.flyWheelMotor.setNeutralMode(NeutralMode.Brake);

		this.elevatorMotor = new WPI_TalonSRX(Constants.ELEVATOR_MOTOR);

		this.elevatorMotor.setNeutralMode(NeutralMode.Brake);

		this.turretWheelMotor = new WPI_TalonSRX(Constants.TURRET_WHEEL_MOTOR);

		this.turretWheelMotor.setNeutralMode(NeutralMode.Brake);

		this.turretSwivelMotor = new WPI_TalonSRX(Constants.TURRET_SWIVEL_MOTOR);

		this.turretSwivelMotor.setNeutralMode(NeutralMode.Brake);

	}

	public void Shoot() {

		this.flyWheelMotor.set(ControlMode.PercentOutput, FLYWHEEL_MOTOR_PERCENT);

	}

	public void ShootStop() {

		this.flyWheelMotor.set(ControlMode.PercentOutput, 0);

	}

	public void ElevatorUp() {

		this.elevatorMotor.set(ControlMode.PercentOutput, ELEVATOR_MOTOR_PERCENT);

	}

	public void ElevatorDown() {

		this.elevatorMotor.set(ControlMode.PercentOutput, -ELEVATOR_MOTOR_PERCENT);

	}

	public void ElevatorStop() {

		this.elevatorMotor.set(ControlMode.PercentOutput, 0);

	}

	public void turretWheelSpin() {

		this.turretWheelMotor.set(ControlMode.PercentOutput, TURRET_WHEEL_MOTOR_PERCENT);

	}

	public void turretWheelStop() {

		this.turretWheelMotor.set(ControlMode.PercentOutput, 0);

	}

	public void turretSwivelLeft() {

		this.turretWheelMotor.set(ControlMode.PercentOutput, TURRET_SWIVEL_MOTOR_PERCENT);

	}

	public void turretSwivelRight() {

		this.turretWheelMotor.set(ControlMode.PercentOutput, -TURRET_SWIVEL_MOTOR_PERCENT);

	}

	public void turretSwivelStop() {

		this.turretSwivelMotor.set(ControlMode.PercentOutput, 0);

	}

}