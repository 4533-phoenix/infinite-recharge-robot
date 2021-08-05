package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.ShooterSystem;

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

	public static Command shootBallCommand() {
		return new FunctionalCommand(
			() -> Robot.shooter.flywheelAndIntakeResetPosition(), 
			() -> Robot.shooter.flywheelAndIntakeOut(),
			(interrupt) -> Robot.shooter.flywheelAndIntakeStop(),
			() -> Robot.shooter.flywheelReachedPosition(3),
			Robot.shooter
		);
	}

	public static Command driveOffLineCommand() {
		return new SequentialCommandGroup(
			driveDistanceCommand(24, Direction.BACKWARD)
		);
	}

	public static Command turretSwivelAuto() {
		return new FunctionalCommand(
			() -> Robot.shooter.word(),
			() -> Robot.shooter.autoTurretSwivel(),
			(interrupt) -> Robot.shooter.turretSwivelStop(),
			() -> Robot.shooter.turretReachedPosition(),
			Robot.shooter
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

	public static Command runFlywheelCommand() {
		return new InstantCommand(
			() -> Robot.shooter.flywheelOut(),
			Robot.shooter
		);
	}

	public static Command testAutoCommand() {
		return new SequentialCommandGroup(
			driveDistanceCommand(60,Direction.FORWARD),
			angleTurnCommand(0.2, 180, Direction.RIGHT),
			driveDistanceCommand(60,Direction.FORWARD)
		);
	}

	public static Command waitFlyTimed() {
		return new WaitCommand(3);
	}

	public static Command driveShootAutoCommand() {
		return new SequentialCommandGroup(
			runFlywheelCommand(),
			waitFlyTimed(),
			shootBallCommand(),
			driveDistanceCommand(24, Direction.BACKWARD)
		);
	}

	public static Command slalomAutoCommand() {
		return new SequentialCommandGroup(
			driveDistanceCommand(30,Direction.FORWARD),
			angleTurnCommand(0.2, 22.5, Direction.LEFT),
			driveDistanceCommand(60 * Math.sqrt(2), Direction.FORWARD),
			angleTurnCommand(0.1, 22.5, Direction.RIGHT),
			driveDistanceCommand(105, Direction.FORWARD),
			angleTurnCommand(0.2, 22.5, Direction.RIGHT),
			driveDistanceCommand(80, Direction.FORWARD),
			angleTurnCommand(0.2, 37.5, Direction.LEFT),
			driveDistanceCommand(30 * Math.sqrt(2), Direction.FORWARD),
			angleTurnCommand(0.2, 60, Direction.LEFT),
			driveDistanceCommand(30 * Math.sqrt(2), Direction.FORWARD),
			angleTurnCommand(0.2, 45, Direction.LEFT),
			driveDistanceCommand(50, Direction.FORWARD),
			angleTurnCommand(0.2, 85, Direction.LEFT),
			driveDistanceCommand(80,Direction.FORWARD),
			angleTurnCommand(0.2, 20, Direction.RIGHT),
			driveDistanceCommand(120, Direction.FORWARD),
			angleTurnCommand(0.2, 30, Direction.RIGHT),
			driveDistanceCommand(60 * Math.sqrt(2), Direction.FORWARD),
			angleTurnCommand(0.2, 22.5, Direction.LEFT),
			driveDistanceCommand(20, Direction.FORWARD)
		);
	}

	public static Command barrelAutoCommand() {
		return new SequentialCommandGroup(
			driveDistanceCommand(100, Direction.FORWARD),
			angleTurnCommand(0.2, 60, Direction.RIGHT),
			driveDistanceCommand(Math.sqrt(1300),Direction.FORWARD),
			angleTurnCommand(0.2,90,Direction.RIGHT),
			driveDistanceCommand(30 * Math.sqrt(2), Direction.FORWARD),
			angleTurnCommand(0.2,90,Direction.RIGHT),
			driveDistanceCommand(30 * Math.sqrt(2), Direction.FORWARD),
			angleTurnCommand(0.2,115,Direction.RIGHT),
			driveDistanceCommand(Math.sqrt(26100),Direction.FORWARD),
			angleTurnCommand(0.2, 90, Direction.LEFT),
			driveDistanceCommand(30 * Math.sqrt(2), Direction.FORWARD),
			angleTurnCommand(0.2, 90, Direction.LEFT),
			driveDistanceCommand(30 * Math.sqrt(2), Direction.FORWARD),
			angleTurnCommand(0.2, 90, Direction.LEFT),
			driveDistanceCommand(120 * Math.sqrt(2), Direction.FORWARD),
			angleTurnCommand(0.2, 90, Direction.LEFT),
			driveDistanceCommand(30 * Math.sqrt(2), Direction.FORWARD),
			angleTurnCommand(0.2, 90, Direction.LEFT),
			driveDistanceCommand(30 * Math.sqrt(2), Direction.FORWARD),
			angleTurnCommand(0.2, 45, Direction.LEFT)
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

	public static Command turretWheelInCommand() {
		return new InstantCommand(
			() -> Robot.shooter.turretWheelIn(),
			Robot.shooter
		);
	}

	// public static Command flywheelRPMCheckDelay() {
	// 	return new WaitCommand(0.02);
	// }

	public static Command turretWheelOutCommand() {
		return new InstantCommand(
			() -> Robot.shooter.turretWheelOut(),
			Robot.shooter
		);
	}

	public static Command turretWheelStopCommand() {
		return new InstantCommand(
			() -> Robot.shooter.turretWheelStop(),
			Robot.shooter
		);
	}

	public static Command flywheelOutCommand() {
		return new InstantCommand(
			() -> Robot.shooter.flywheelOut(),
			Robot.shooter
		);
	}

	public static Command flywheelInCommand() {
		return new InstantCommand(
			() -> Robot.shooter.flywheelIn(),
			Robot.shooter
		);
	}

	public static Command flywheelStopCommand() {
		return new InstantCommand(
			() -> Robot.shooter.flywheelStop(),
			Robot.shooter
		);
	}

	public static Command turretSwivelLeftCommand() {
		return new InstantCommand(
			() -> Robot.shooter.turretSwivelLeft(),
			Robot.shooter
		);
	}

	public static Command turretSwivelRightCommand() {
		return new InstantCommand(
			() -> Robot.shooter.turretSwivelRight(),
			Robot.shooter
		);
	}

	public static Command turretSwivelStopCommand() {
		return new InstantCommand(
			() -> Robot.shooter.turretSwivelStop(),
			Robot.shooter
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

	public static Command climbDownCommand() {
		return new InstantCommand(
			() -> Robot.climber.climbDown(),
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

