package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ClimbSystem;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.IntakeSystem;

public class CommandFactory {

	public CommandFactory() {
	}

	public static Command driveDistanceCommand(double distance, Direction direction, DriveSystem driveSystem) {
		return new FunctionalCommand(
			() -> driveSystem.resetPosition(),
			() -> driveSystem.driveDistance(distance, direction),
			(interrupt) -> driveSystem.tank(0, 0),
			() -> driveSystem.reachedPosition(),
			driveSystem
		);
	}

	public static Command driveCurveCommand(double left, double right, Direction direction, DriveSystem driveSystem) {
		return new FunctionalCommand(
			() -> driveSystem.resetPosition(),
			() -> driveSystem.driveCurve(left, right, direction),
			(interrupt) -> driveSystem.tank(0, 0),
			() -> driveSystem.reachedCurve(left, right),
			driveSystem
		);
	}

	public static Command angleTurnCommand(double speed, double angle, Direction direction, DriveSystem driveSystem) {
		return new FunctionalCommand(
			() -> driveSystem.resetAngle(),
			() -> driveSystem.turn(speed, direction),
			(interrupt) -> driveSystem.tank(0, 0),
			() -> driveSystem.getAngle() >= angle,
			driveSystem
		);
	}

	public static Command intakeInCommand(IntakeSystem intakeSystem) {
		return new InstantCommand(
			() -> intakeSystem.intakeIn(),
			intakeSystem
		);
	}

	public static Command intakeOutCommand(IntakeSystem intakeSystem) {
		return new InstantCommand(
			() -> intakeSystem.intakeOut(),
			intakeSystem
		);
	}

	public static Command intakeStopCommand(IntakeSystem intakeSystem) {
		return new InstantCommand(
			() -> intakeSystem.intakeStop(),
			intakeSystem
		);
	}

	public static Command conveyorInCommand(IntakeSystem intakeSystem) {
		return new InstantCommand(
			() -> intakeSystem.conveyorIn(),
			intakeSystem
		);
	}

	public static Command conveyorOutCommand(IntakeSystem intakeSystem) {
		return new InstantCommand(
			() -> intakeSystem.conveyorOut(),
			intakeSystem
		);
	}

	public static Command emptyConveyorCommand(IntakeSystem intakeSystem) {
		return new InstantCommand(
			() -> intakeSystem.enmptyConveyor(),
			intakeSystem
		);
	}

	public static Command conveyorStopCommand(IntakeSystem intakeSystem){
		return new InstantCommand(
			() -> intakeSystem.conveyorStop(),
			intakeSystem
		);
	}

	public static Command hookUpCommand(ClimbSystem climbSystem){
		return new InstantCommand(
			() -> climbSystem.hookUp(),
			climbSystem
		);
	}

	public static Command hookDownCommand(ClimbSystem climbSystem) {
		return new InstantCommand(
			() -> climbSystem.hookDown(),
			climbSystem
		);
	}

	public static Command hookStopCommand(ClimbSystem climbSystem) {
		return new InstantCommand(
			() -> climbSystem.hookStop(),
			climbSystem
		);
	}

	public static Command climbCommand(ClimbSystem climbSystem) {
		return new InstantCommand(
			() -> climbSystem.climb(),
			climbSystem
		);
	}
}