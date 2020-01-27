package frc.robot.commands;

import frc.robot.subsystems.DriveSystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

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
    public static Command angleTurnCommand(double speed, int angle, Direction direction, DriveSystem driveSystem) {
        return new FunctionalCommand(
            ()-> driveSystem.resetAngle(), 
            ()-> driveSystem.turn(speed, direction), 
            (interrupt)-> driveSystem.tank(0,0), 
            ()-> driveSystem.getAngle() >= angle, 
            driveSystem
          );
    }
}