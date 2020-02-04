package frc.robot.commands;

import frc.robot.subsystems.DriveSystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import java.util.List;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;



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
    public static Command angleTurnCommand(double speed, double angle, Direction direction, DriveSystem driveSystem) {
        return new FunctionalCommand(
            ()-> driveSystem.resetAngle(), 
            ()-> driveSystem.turn(speed, direction), 
            (interrupt)-> driveSystem.tank(0,0), 
            ()-> driveSystem.getAngle() >= angle, 
            driveSystem
          );
    }
    public static Command getTrajectoryCommand(DriveSystem driveSystem) {
        DifferentialDriveVoltageConstraint constraint = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(DriveSystem.kS, DriveSystem.kV, DriveSystem.kA),
            DriveSystem.kDriveKinematics,
            10
        );
        
        TrajectoryConfig config = new TrajectoryConfig(DriveSystem.kMaxSpeed, DriveSystem.kMaxAcceleration)
        .setKinematics(DriveSystem.kDriveKinematics)
        .addConstraint(constraint);

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                new Translation2d(-5, 1)
                //new Translation2d(2, -1)
            ),
            new Pose2d(0, 0, new Rotation2d(90)),
            config
        );
        RamseteCommand command = new RamseteCommand(
            trajectory,
            driveSystem::getPose,
            new RamseteController(DriveSystem.kRamseteB, DriveSystem.kRamseteZeta),
            new SimpleMotorFeedforward(DriveSystem.kS, DriveSystem.kV, DriveSystem.kA),
            DriveSystem.kDriveKinematics,
            driveSystem::getWheelSpeeds,
            new PIDController(DriveSystem.kPVelocity, 0, 0),
            new PIDController(DriveSystem.kPVelocity, 0, 0),
            driveSystem::tankVoltage,
            // (l, r) -> driveSystem.tankVoltage(l, r),
            driveSystem
        );
        return command;


    }
}