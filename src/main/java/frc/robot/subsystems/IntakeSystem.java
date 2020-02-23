package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSystem extends SubsystemBase {

	private final double INTAKE_MOTOR_PERCENT = 0.5;

	private WPI_TalonSRX intakeMotor;

	private DigitalInput powerCell1;
	private DigitalInput powerCell2;
	private DigitalInput powerCell3;
	private DigitalInput powerCell4;
	private DigitalInput powerCell5;

	public IntakeSystem() {
		this.intakeMotor = new WPI_TalonSRX(Constants.INTAKE_MOTOR);


		this.powerCell1 = new DigitalInput(Constants.POWER_CELL_1);
		this.powerCell2 = new DigitalInput(Constants.POWER_CELL_2);
		this.powerCell3 = new DigitalInput(Constants.POWER_CELL_3);
		this.powerCell4 = new DigitalInput(Constants.POWER_CELL_4);
		this.powerCell5 = new DigitalInput(Constants.POWER_CELL_5);
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
	/**
	 * Get the sensor values for the powercells.
	 *
	 * @return an array of booleans representing the state of all the powercell
	 * sensors. When a powercell is present the value is <code>true</code>,
	 * otherwise it is <code>false</code>.
	 */
	public boolean[] getPowerCells() {
		return new boolean[] {
			this.powerCell1.get(),
			this.powerCell2.get(),
			this.powerCell3.get(),
			this.powerCell4.get(),
			this.powerCell5.get()
		};
	}

	public boolean hasPowerCell() {
		return !this.powerCell1.get();
	}

	@Override
	public void periodic() {
	}
}
