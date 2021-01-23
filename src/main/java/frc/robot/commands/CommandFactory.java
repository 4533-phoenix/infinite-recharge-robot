package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;

public class CommandFactory {

	public CommandFactory() {
	}

	public static Command driveDistanceCommand(double distance, Direction direction) {
		return new FunctionalCommand(
			() -> Robot.drive.resetPosition(),
			() -> Robot.drive.driveDistance(distance, direction),
			(interrupt) -> Robot.drive.tank(0, 0),
			() -> Robot.drive.reachedPosition(),
			Robot.drive
		);
	}

	public static Command driveCurveCommand(double left, double right, Direction direction) {
		return new FunctionalCommand(
			() -> Robot.drive.resetPosition(),
			() -> Robot.drive.driveCurve(left, right, direction),
			(interrupt) -> Robot.drive.tank(0, 0),
			() -> Robot.drive.reachedCurve(left, right),
			Robot.drive
		);
	}

	public static Command driveCircleCommand(double speed, double angle, Direction direction, double radius) {
		return new FunctionalCommand(
			() -> Robot.drive.resetPosition(),
			() -> Robot.drive.driveCircle(speed, angle, direction, radius),
			(interrupt) -> Robot.drive.tank(0,0),
			() -> Robot.drive.reachedCircle(angle, radius, direction),
			Robot.drive
		);
	}

	public static Command angleTurnCommand(double speed, double angle, Direction direction) {
		return new FunctionalCommand(
			() -> Robot.drive.resetAngle(),
			() -> Robot.drive.turn(speed, direction),
			(interrupt) -> Robot.drive.tank(0, 0),
			() -> Robot.drive.getAngle() >= angle,
			Robot.drive
		);
	}

	public static Command intakeInCommand() {
		return new InstantCommand(
			() -> Robot.intake.intakeIn(),
			Robot.intake
		);
	}

	public static Command intakeOutCommand() {
		return new InstantCommand(
			() -> Robot.intake.intakeOut(),
			Robot.intake
		);
	}

	public static Command intakeStopCommand() {
		return new InstantCommand(
			() -> Robot.intake.intakeStop(),
			Robot.intake
		);
	}

}

