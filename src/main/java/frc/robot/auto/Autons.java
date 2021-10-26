package frc.robot.auto;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.HSSwerveDriveController;
import frc.robot.commands.intake.IntakeAutonControlForward;
import frc.robot.commands.intake.MoveBallsToShooter;
import frc.robot.commands.shooter.ShootWithHighHood;
import frc.robot.commands.shooter.ShootWithLimelight;
import frc.robot.commands.shooter.ShooterVelocityManual;

public class Autons {
    // public static SequentialCommandGroup autoPath1 = new SequentialCommandGroup(
    //     new ParallelCommandGroup(
    //         new HSSwerveDriveController(Trajectories.goToTrench, Rotation2d.fromDegrees(0))), 
    //     new HSSwerveDriveController(Trajectories.returnFromTrench, Rotation2d.fromDegrees(0)));

    public static SequentialCommandGroup stealBallsFromTrench = new SequentialCommandGroup(
        new HSSwerveDriveController(Trajectories.chezy_moveToBalls, Rotation2d.fromDegrees(0)),
        new ParallelCommandGroup(
            new HSSwerveDriveController(Trajectories.chezy_moveToShootingLocation, Rotation2d.fromDegrees(0)),
            new ShooterVelocityManual(65)
        ),
        new ParallelCommandGroup(
            new ShootWithHighHood(),
            new MoveBallsToShooter()
        )
    );

    public static SequentialCommandGroup leftToUpAgainstGoal = new SequentialCommandGroup(
        new ParallelCommandGroup(
            new HSSwerveDriveController(Trajectories.chezy_leftInitiationToScoringZone, Rotation2d.fromDegrees(0)),
            new ShooterVelocityManual(65)
        ),
        new ParallelCommandGroup(
            new ShootWithHighHood(),
            new MoveBallsToShooter()
        )
    );

    public static SequentialCommandGroup centerToUpAgainstGoal = new SequentialCommandGroup(
        new ParallelCommandGroup(
            new HSSwerveDriveController(Trajectories.chezy_centerInitiationToScoringZone, Rotation2d.fromDegrees(0)),
            new ShooterVelocityManual(65)
        ),
        new ParallelCommandGroup(
            new ShootWithHighHood(),
            new MoveBallsToShooter()
        )
    );

    public static SequentialCommandGroup rightToUpAgainstGoal = new SequentialCommandGroup(
        new ParallelCommandGroup(
            new HSSwerveDriveController(Trajectories.chezy_rightInitiationToScoringZone, Rotation2d.fromDegrees(0)),
            new ShooterVelocityManual(65)
        ),
        new ParallelCommandGroup(
            new ShootWithHighHood(),
            new MoveBallsToShooter()
        )
    );

    public static SequentialCommandGroup throughTrench = new SequentialCommandGroup(
        new HSSwerveDriveController(Trajectories.chezy_setUpLeftTrench, Rotation2d.fromDegrees(0)),
        new ParallelCommandGroup(
            new HSSwerveDriveController(Trajectories.chezy_moveThroughTrench, Rotation2d.fromDegrees(0)),
            new IntakeAutonControlForward(0.5)
        ),
        
        new HSSwerveDriveController(Trajectories.chezy_moveBackThroughTrench, Rotation2d.fromDegrees(0)),
        new ParallelCommandGroup(
            new HSSwerveDriveController(Trajectories.chezy_shootTrenchBalls, Rotation2d.fromDegrees(0)),
            new ShooterVelocityManual(65)
        ),
        new ParallelCommandGroup(
            new ShootWithHighHood(),
            new MoveBallsToShooter()
        )

    );

    public static SequentialCommandGroup autonCommand = centerToUpAgainstGoal;


}
