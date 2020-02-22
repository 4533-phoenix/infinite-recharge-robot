package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import jdk.dynalink.linker.ConversionComparator;

public class IntakeSystem extends SubsystemBase {
	/**
	 * Creates a new IntakeSystem.
	 */
	private WPI_TalonSRX intakeMotor;
	private WPI_TalonSRX conveyorMotor;

	private DigitalInput powerCell1;
	private DigitalInput powerCell2;
	private DigitalInput powerCell3;
	private DigitalInput powerCell4;
	private DigitalInput powerCell5;

	private final double INTAKE_MOTOR_PERCENT = 0.65;
	private final double CONVEYOR_MOTOR_PERCENT = 0.5;

	public IntakeSystem() {
		intakeMotor = new WPI_TalonSRX(Constants.INTAKE_MOTOR);
		conveyorMotor = new WPI_TalonSRX(Constants.CONVEYOR_MOTOR);
		powerCell1 = new DigitalInput(Constants.POWER_CELL_1);
		powerCell2 = new DigitalInput(Constants.POWER_CELL_2);
		powerCell3 = new DigitalInput(Constants.POWER_CELL_3);
		powerCell4 = new DigitalInput(Constants.POWER_CELL_4);
		powerCell5 = new DigitalInput(Constants.POWER_CELL_5);
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

	public void conveyorOut() {
		this.conveyorMotor.set(ControlMode.PercentOutput, CONVEYOR_MOTOR_PERCENT);
	}

	public void conveyorIn(){
		this.conveyorMotor.set(ControlMode.PercentOutput, -CONVEYOR_MOTOR_PERCENT);
	}

	public void enmptyConveyor() {
		this.conveyorMotor.set(ControlMode.PercentOutput, -1.0);
	}

	public void conveyorStop() {
		this.conveyorMotor.set(ControlMode.PercentOutput, 0);
	}

	public boolean getPowerCell1(){
		return powerCell1.get();
	}

	public boolean getPowerCell2(){
		return powerCell2.get();
	}

	public boolean getPowerCell3(){
		return powerCell3.get();
	}

	public boolean getPowerCell4(){
		return powerCell4.get();
	}

	public boolean getPowerCell5(){
		return powerCell5.get();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
