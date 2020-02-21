package frc.robot;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveSystem;

public class Robot extends TimedRobot {
	private Logger logger = LogManager.getLogger(Robot.class.getName());

	private ObjectMapper mapper = new ObjectMapper();

	private PowerDistributionPanel pdp = new PowerDistributionPanel();

	private Command m_autonomousCommand = null;

	private RobotContainer container = null;

	private RobotState robotState = null;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		// Instantiate our RobotContainer.  This will perform all our button
		// bindings, and put our autonomous chooser on the dashboard.
		this.container = new RobotContainer();

		this.robotState = new RobotState()
			.withPDP(new PowerDistributionPanel())
			.withDriveSystem(container.getDriveSystem())
			.withIntakeSystem(container.getIntakeSystem());
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		this.robotState.update();
		try {
			String state = this.mapper.writeValueAsString(this.robotState);
			this.logger.info(state);
		} catch (JsonProcessingException e) {
			this.logger.error(e.toString());
		}
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 */
	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
	}

	/**
	 * This autonomous runs the autonomous command selected by your
	 * {@link RobotContainer} class.
	 */
	@Override
	public void autonomousInit() {
		DriveSystem driveSystem =  this.container.getDriveSystem();
		driveSystem.resetAngle();
		driveSystem.resetPosition();

		m_autonomousCommand =  this.container.getAutonomousCommand("");

		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
		}
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {

	}

	@Override
	public void teleopInit() {
		DriveSystem driveSystem = this.container.getDriveSystem();
		driveSystem.resetAngle();
		driveSystem.resetPosition();

		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
	}

	@Override
	public void testInit() {
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}
