package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;


public class ControlPanelSystem extends SubsystemBase {

	private static final double TICKS_PER_ROTATION = 4096.0;
	private static final double PANEL_DIAMETER = 32.0;
	private static final double WHEEL_DIAMETER = 2.0;
	private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
	private static final double SPINS_PER_ROTATION = PANEL_DIAMETER / WHEEL_DIAMETER;
	private static final double TICKS_PER_INCH = TICKS_PER_ROTATION / WHEEL_CIRCUMFERENCE;

	private static double targetPosition = 0;

	private final double CP_MOTOR_PERCENT = 0.5;

	private WPI_TalonSRX controlPanelMotor;

	public ControlPanelSystem() {
		this.controlPanelMotor = new WPI_TalonSRX(Constants.CONTROL_PANEL_MOTOR);

		this.controlPanelMotor.setNeutralMode(NeutralMode.Brake);
	}

	public void spinControlPanel(int rotations) {
		double wheelSpins = SPINS_PER_ROTATION * rotations;
		targetPosition = WHEEL_CIRCUMFERENCE * wheelSpins * TICKS_PER_INCH;

		this.controlPanelMotor.set(ControlMode.Position, targetPosition);
	}

	public void stopControlPanel() {
		this.controlPanelMotor.set(ControlMode.PercentOutput, 0);
	}

	public boolean reachedCPPosition() {
		double rotationPos = this.controlPanelMotor.getSelectedSensorPosition();

		return (rotationPos >= targetPosition);
	}
}