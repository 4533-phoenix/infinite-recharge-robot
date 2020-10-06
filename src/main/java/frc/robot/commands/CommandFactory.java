package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSystem;

public class CommandFactory {

	public CommandFactory() {
	}

	public static Command driveDistanceCommand(double distance, Direction direction) {
		return new FunctionalCommand(
			() -> {
				Robot.drive.setPIDF(
					DriveSystem.POSITION_P,
					DriveSystem.POSITION_I,
					DriveSystem.POSITION_D,
					DriveSystem.POSITION_FEED_FORWARD
				);
				Robot.drive.resetPosition();
			},
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

	public static Command angleTurnCommand(double speed, double angle, Direction direction) {
		return new FunctionalCommand(
			() -> {
				Robot.drive.setPIDF(
					DriveSystem.VELOCITY_P,
					DriveSystem.VELOCITY_I,
					DriveSystem.VELOCITY_D,
					DriveSystem.VELOCITY_FEED_FORWARD
				);
				Robot.drive.resetAngle();
			},
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

	public static Command conveyorInCommand() {
		return new InstantCommand(
			() -> Robot.conveyor.forward(),
			Robot.conveyor
		);
	}

	public static Command conveyorOutCommand() {
		return new InstantCommand(
			() -> Robot.conveyor.reverse(),
			Robot.conveyor
		);
	}

	public static Command emptyConveyorCommand() {
		return new InstantCommand(
			() -> Robot.conveyor.empty(),
			Robot.conveyor
		);
	}

	public static Command conveyorStopCommand(){
		return new InstantCommand(
			() -> Robot.conveyor.stop(),
			Robot.conveyor
		);
	}

	public static Command hookUpCommand(){
		return new InstantCommand(
			() -> Robot.climber.hookUp(),
			Robot.climber
		);
	}

	public static Command hookDownCommand() {
		return new InstantCommand(
			() -> Robot.climber.hookDown(),
			Robot.climber
		);
	}

	public static Command hookStopCommand() {
		return new InstantCommand(
			() -> Robot.climber.hookStop(),
			Robot.climber
		);
	}

	public static Command climbCommand() {
		return new InstantCommand(
			() -> Robot.climber.climb(),
			Robot.climber
		);
	}

	public static Command climbStopCommand() {
		return new InstantCommand(
			() -> Robot.climber.climbStop(),
			Robot.climber
		);
	}
}