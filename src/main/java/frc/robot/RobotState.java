package frc.robot;

import com.fasterxml.jackson.annotation.JsonIgnore;
import com.fasterxml.jackson.annotation.JsonProperty;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.IntakeSystem;

public class RobotState {

	@JsonIgnore
	private PowerDistributionPanel pdp = null;

	@JsonIgnore
	private DriveSystem driveSystem = null;

	@JsonIgnore
	private IntakeSystem intakeSystem = null;

	@JsonProperty
	private long timestamp = 0;

	@JsonProperty
	private double[] current = new double[16];

	@JsonProperty
	private double totalCurrent = 0.0;

	@JsonProperty
	private double voltage = 0.0;

	@JsonProperty
	private double rightSpeed = 0.0;

	@JsonProperty
	private double leftSpeed = 0.0;

	@JsonProperty
	private double rightDistance = 0.0;

	@JsonProperty
	private double leftDistance = 0.0;

	@JsonProperty
	private boolean[] powerCells = new boolean[5];

	@JsonProperty
	private double pdpTemperature = 0.0;

	public RobotState() {}

	public RobotState withPDP(PowerDistributionPanel pdp) {
		this.pdp = pdp;
		return this;
	}

	public RobotState withDriveSystem(DriveSystem driveSystem) {
		this.driveSystem = driveSystem;
		return this;
	}

	public RobotState withIntakeSystem(IntakeSystem intakeSystem) {
		this.intakeSystem = intakeSystem;
		return this;
	}

	public void update() {
		this.timestamp = System.currentTimeMillis();

		if (this.pdp != null) {
			for (int channel = 0; channel < 16; channel++) {
				this.current[channel] = this.pdp.getCurrent(channel);
			}

			this.totalCurrent = this.pdp.getTotalCurrent();
			this.voltage = this.pdp.getVoltage();
			this.pdpTemperature = this.pdp.getTemperature();
		}

		if (this.driveSystem != null) {
			DifferentialDriveWheelSpeeds wheelSpeeds = this.driveSystem.getWheelSpeeds();
			this.rightSpeed = wheelSpeeds.rightMetersPerSecond;
			this.leftSpeed = wheelSpeeds.leftMetersPerSecond;

			this.rightDistance = this.driveSystem.getRightDistance();
			this.leftDistance = this.driveSystem.getLeftDistance();
		}

		if (this.intakeSystem != null) {
		}
	}
}