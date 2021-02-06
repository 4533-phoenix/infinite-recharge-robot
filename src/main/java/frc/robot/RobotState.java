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
	private double pdpTemperature = 0.0;

	public RobotState() {
		this.pdp = new PowerDistributionPanel();
		this.driveSystem = Robot.drive;
		this.intakeSystem = Robot.intake;
	}

	public void update() {
		this.timestamp = System.currentTimeMillis();

		// Power Distribution
		if (this.pdp != null) {
			this.totalCurrent = this.pdp.getTotalCurrent();
			this.voltage = this.pdp.getVoltage();
			this.pdpTemperature = this.pdp.getTemperature();
		}

		// Drive System
		if (this.driveSystem != null) {
			DifferentialDriveWheelSpeeds wheelSpeeds = this.driveSystem.getWheelSpeeds();
			this.rightSpeed = wheelSpeeds.rightMetersPerSecond;
			this.leftSpeed = wheelSpeeds.leftMetersPerSecond;

			this.rightDistance = this.driveSystem.getRightDistance();
			this.leftDistance = this.driveSystem.getLeftDistance();
		}

		// Intake System
		if (this.intakeSystem != null) {
		}
	}
}