package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.DriveSystem;

public class CommandFactory {

	public CommandFactory() {
	}

	public static Command driveDistanceCommand(double distance, Direction direction, DriveSystem driveSystem) {
		return new FunctionalCommand(() -> driveSystem.resetPosition(),
				() -> driveSystem.driveDistance(distance, direction), (interrupt) -> driveSystem.tank(0, 0),
				() -> driveSystem.reachedPosition(), driveSystem);
	}

	public static Command driveCurveCommand(double left, double right, Direction direction, DriveSystem driveSystem) {
		System.out.println("HERE");

		return new FunctionalCommand(
			() -> driveSystem.resetPosition(),
			() -> driveSystem.driveCurve(left, right, direction),
			(interrupt) -> driveSystem.tank(0, 0),
			() -> driveSystem.reachedCurve(left, right),
			driveSystem
		);
	}
	public static Command angleTurnCommand(double speed, double angle, Direction direction, DriveSystem driveSystem) {
		return new FunctionalCommand(() -> driveSystem.resetAngle(), () -> driveSystem.turn(speed, direction),
				(interrupt) -> driveSystem.tank(0, 0), () -> driveSystem.getAngle() >= angle, driveSystem);
	}
}