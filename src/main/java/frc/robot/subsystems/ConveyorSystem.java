package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ConveyorSystem extends SubsystemBase {
	private static final double PEAK_OUTPUT_FORWARD = 1.0;
	private static final double PEAK_OUTPUT_REVERSE = -1.0;

	private static final double NOMINAL_OUTPUT_FORWARD = 0.0;
	private static final double NOMINAL_OUTPUT_REVERSE = 0.0;

	private final static double CONVEYOR_MOTOR_PERCENT = 0.5;

	private WPI_TalonSRX motor = new WPI_TalonSRX(Constants.CONVEYOR_MOTOR);

	public ConveyorSystem() {
		this.motor.configSelectedFeedbackSensor(
			FeedbackDevice.CTRE_MagEncoder_Absolute,
			0,
			30);

		this.motor.configPeakOutputForward(PEAK_OUTPUT_FORWARD);
		this.motor.configPeakOutputReverse(PEAK_OUTPUT_REVERSE);

		this.motor.configNominalOutputForward(NOMINAL_OUTPUT_FORWARD);
		this.motor.configNominalOutputReverse(NOMINAL_OUTPUT_REVERSE);
	}

	public void conveyorOut() {
		this.motor.set(ControlMode.PercentOutput, CONVEYOR_MOTOR_PERCENT);
	}

	public void conveyorIn(){
		this.motor.set(ControlMode.PercentOutput, -CONVEYOR_MOTOR_PERCENT);
	}

	public void enmptyConveyor() {
		this.motor.set(ControlMode.PercentOutput, -1.0);
	}

	public void conveyorStop() {
		this.motor.set(ControlMode.PercentOutput, 0);
	}

	public void conveyorStep() {
		this.motor.set(ControlMode.Position, 4096);
	}
}