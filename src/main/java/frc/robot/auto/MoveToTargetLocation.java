package frc.robot.auto;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.SwerveDriveKinematicsConstraint;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;

public class MoveToTargetLocation
{
    static TrajectoryConfig config = new TrajectoryConfig(Drivetrain.AUTO_MAX_SPEED / 1.5, Drivetrain.AUTO_MAX_SPEED_ACCELERATION / 1.5)
                                                         .addConstraint(new SwerveDriveKinematicsConstraint(Drivetrain.getInstance().getKinematics(), Drivetrain.AUTO_MAX_SPEED));


    public static Trajectory getToTargetTrajectory(){
        double xmove = Shooter.getInstance().getXDistance();
        double ymove = Shooter.getInstance().getYDistance();

        xmove*=0.3048;
        ymove*=0.3048;
        return TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(270)),
                new Pose2d(xmove, ymove, Rotation2d.fromDegrees(270))),
                config);
    }
         
}