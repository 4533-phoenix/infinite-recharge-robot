package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Direction;

import com.ctre.phoenix.motorcontrol.NeutralMode;

public class ShooterSystem extends SubsystemBase {
	
	private final double FLYWHEEL_MOTOR_PERCENT = 1;

	private final double ELEVATOR_MOTOR_PERCENT = 0.5;

	private final double TURRET_WHEEL_MOTOR_PERCENT = 0.5;

	private final double TURRET_SWIVEL_MOTOR_PERCENT = 0.1;

	private WPI_TalonFX flywheelMotorRight;
	private WPI_TalonFX flywheelMotorLeft;

	private WPI_VictorSPX turretWheelMotor;

	private WPI_TalonSRX turretSwivelMotor;

	private WPI_TalonSRX elevatorMotor;

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

	}

	public void Shoot() {

		this.flywheelMotorRight.set(ControlMode.PercentOutput, FLYWHEEL_MOTOR_PERCENT);
		this.flywheelMotorLeft.set(ControlMode.PercentOutput, FLYWHEEL_MOTOR_PERCENT);

	}

	public void ShootStop() {

		this.flywheelMotorRight.set(ControlMode.PercentOutput, 0);
		this.flywheelMotorLeft.set(ControlMode.PercentOutput, 0);

	}

	public void turretWheelIn() {
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

}
