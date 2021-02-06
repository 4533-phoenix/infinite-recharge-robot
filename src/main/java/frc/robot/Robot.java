package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.IntakeSystem;

public class Robot extends TimedRobot {
	/**
	 * The robot's configured autonomous command.
	 */
	private Command autoCommand = null;

	// TODO: While it's purpose and intention is understood, the 'container'
	// concept still doesn't sit well. It would be ideal if we could determine a
	// better approach and take that instead. This is a just a note to remind us
	// to consider it as we move forward.
	private RobotContainer container = null;

	/**
	 * The robot's drive train subsystem.
	 */
	public final static DriveSystem drive = new DriveSystem();

	/**
	 * The robot's intake subsystem.
	 */
	public final static IntakeSystem intake = new IntakeSystem();

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		// Instantiate our RobotContainer.  This will perform all our button
		// bindings, and put our autonomous chooser on the dashboard.
		this.container = new RobotContainer();
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
		Robot.drive.setPIDF(
			DriveSystem.POSITION_P,
			DriveSystem.POSITION_I,
			DriveSystem.POSITION_D,
			DriveSystem.POSITION_FEED_FORWARD
		);

		Robot.drive.resetAngle();
		Robot.drive.resetPosition();

		this.autoCommand = this.container.getAutonomousCommand("score_trench");

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
		Robot.drive.setPIDF(
			DriveSystem.VELOCITY_P,
			DriveSystem.VELOCITY_I,
			DriveSystem.VELOCITY_D,
			DriveSystem.VELOCITY_FEED_FORWARD
		);

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
