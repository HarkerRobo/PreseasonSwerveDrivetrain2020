package frc.robot.auto;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.SwerveDriveKinematicsConstraint;
import frc.robot.subsystems.Drivetrain;

public class Trajectories {
    static TrajectoryConfig config = new TrajectoryConfig(Drivetrain.AUTO_MAX_SPEED, Drivetrain.AUTO_MAX_SPEED_ACCELERATION)
                                                         .addConstraint(new SwerveDriveKinematicsConstraint(Drivetrain.getInstance().getKinematics(), Drivetrain.AUTO_MAX_SPEED));

    public static Trajectory moveForward = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), 
                    new Pose2d(3, 0, Rotation2d.fromDegrees(0))),
            config);

    public static Trajectory rightAndForward = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), 
                    new Pose2d(2, 0, Rotation2d.fromDegrees(0)), 
                    new Pose2d(3, 2, Rotation2d.fromDegrees(0))),
            config);
     public static Trajectory sShaped = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(-90)), 
                    new Pose2d(2, 0, Rotation2d.fromDegrees(90)), 
                    new Pose2d(4, 0, Rotation2d.fromDegrees(-90))),
            config);
}
