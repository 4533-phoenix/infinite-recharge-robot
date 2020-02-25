package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSystem extends SubsystemBase {

	private final double INTAKE_MOTOR_PERCENT = 0.5;

	private WPI_TalonSRX intakeMotor;

	public IntakeSystem() {
		this.intakeMotor = new WPI_TalonSRX(Constants.INTAKE_MOTOR);
}

	public void intakeIn() {
		this.intakeMotor.set(ControlMode.PercentOutput, INTAKE_MOTOR_PERCENT);
	}

	public void intakeOut() {
		this.intakeMotor.set(ControlMode.PercentOutput, -INTAKE_MOTOR_PERCENT);
	}

	public void intakeStop(){
		this.intakeMotor.set(ControlMode.PercentOutput, 0);
	}

	@Override
	public void periodic() {
	}
}
