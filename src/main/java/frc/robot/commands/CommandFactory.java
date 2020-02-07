package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DriveSystem;

public class CommandFactory {

    public CommandFactory() {}

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
        // Define the voltage constraints for the trajectory.
        DifferentialDriveVoltageConstraint constraint =
            new DifferentialDriveVoltageConstraint(
                DriveSystem.FEED_FORWARD,
                DriveSystem.KINEMATICS,
                10
            );

        // Define the trajectory configuration.
        TrajectoryConfig config =
            new TrajectoryConfig(DriveSystem.kMaxSpeed, DriveSystem.kMaxAcceleration)
                .setKinematics(DriveSystem.KINEMATICS)
                .addConstraint(constraint);

        // Define and generate the trajectory.
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                new Translation2d(1, 2),
                new Translation2d(3, 2)
            ),
            new Pose2d(0, 4, new Rotation2d(0)),
            config
        );

        PIDController leftController = new PIDController(
            DriveSystem.VELOCITY_P,
            DriveSystem.VELOCITY_I,
            DriveSystem.VELOCITY_D
        );

        PIDController rightController = new PIDController(
            DriveSystem.VELOCITY_P,
            DriveSystem.VELOCITY_I,
            DriveSystem.VELOCITY_D
        );

        RamseteCommand command = new RamseteCommand(
            trajectory,
            driveSystem::getPose,
            new RamseteController(),
            DriveSystem.FEED_FORWARD,
            DriveSystem.KINEMATICS,
            driveSystem::getWheelSpeeds,
            leftController,
            rightController,
            driveSystem::tankVoltage,
            driveSystem
        );

        return command;
    }
}