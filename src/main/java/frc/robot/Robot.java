package frc.robot;

import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ClimbSystem;
import frc.robot.subsystems.ConveyorSystem;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.IntakeSystem;

public class Robot extends TimedRobot {
	private Logger logger = LogManager.getLogger(Robot.class.getName());

	private ObjectMapper mapper = new ObjectMapper();

	private Command autoCommand = null;

	private RobotContainer container = null;

	private RobotState robotState = null;

	private ScheduledThreadPoolExecutor executor = null;

	public final static DriveSystem drive = new DriveSystem();

	public final static IntakeSystem intake = new IntakeSystem();

	public final static ConveyorSystem conveyor = new ConveyorSystem();

	public final static ClimbSystem climber = new ClimbSystem();

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		// Instantiate our RobotContainer.  This will perform all our button
		// bindings, and put our autonomous chooser on the dashboard.
		this.container = new RobotContainer();

		this.executor = new ScheduledThreadPoolExecutor(2);

		this.robotState = new RobotState()
			.withPDP(new PowerDistributionPanel())
			.withDriveSystem(Robot.drive)
			.withIntakeSystem(Robot.intake);

		this.executor.scheduleAtFixedRate(
			() -> {
				this.robotState.update();
				try {
					String state = this.mapper.writeValueAsString(this.robotState);
					this.logger.info(state);
				} catch (JsonProcessingException e) {
					this.logger.error(e.toString());
				}
			},
			0,   // initial delay
			100, // delay
			TimeUnit.MILLISECONDS
		);
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
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
		Robot.drive.resetAngle();
		Robot.drive.resetPosition();

		this.autoCommand = this.container.getAutonomousCommand("");

		// schedule the autonomous command (example)
		if (this.autoCommand != null) {
			this.autoCommand.schedule();
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
		Robot.drive.resetAngle();
		Robot.drive.resetPosition();

		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (this.autoCommand != null) {
			this.autoCommand.cancel();
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

	@Override
	public void startCompetition() {
		super.startCompetition();
	}
}
