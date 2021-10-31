package frc.robot.auto;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drivetrain.HSSwerveDriveController;
import frc.robot.commands.drivetrain.RotateInPlace;
import frc.robot.commands.drivetrain.RotateToAngle;
import frc.robot.commands.intake.IntakeAutonControlForward;
import frc.robot.commands.intake.MoveBallsToShooter;
import frc.robot.commands.shooter.Rev;
import frc.robot.commands.shooter.ShootWithHighHood;
import frc.robot.commands.shooter.ShootWithLimelight;
import frc.robot.commands.shooter.ShooterVelocityManual;
import frc.robot.subsystems.Intake;

public class Autons {
    // public static SequentialCommandGroup autoPath1 = new SequentialCommandGroup(
        
    //     new ParallelCommandGroup(
    //         new HSSwerveDriveController(Trajectories.goToTrench, Rotation2d.fromDegrees(0))), 
    //     new HSSwerveDriveController(Trajectories.returnFromTrench, Rotation2d.fromDegrees(0)));

    public static SequentialCommandGroup stealBallsFromTrench = new SequentialCommandGroup(
        new InstantCommand(() -> {
            Intake.getInstance().getSolenoid().set(Value.kReverse);
        }),
        
        new ParallelRaceGroup(
            new HSSwerveDriveController(Trajectories.chezy_moveToBalls, Rotation2d.fromDegrees(0)),
            new IntakeAutonControlForward(0.8)
        ),
        new ParallelRaceGroup(
            new HSSwerveDriveController(Trajectories.chezy_moveToShootingLocation, Rotation2d.fromDegrees(0)),
            new Rev(70)
        ),
        
        new ParallelDeadlineGroup(
            new WaitCommand(0.5),
            new RotateToAngle(15)
        ),

        new ParallelDeadlineGroup(
            new WaitCommand(0.5),
            new RotateInPlace()
        ),

        new ParallelDeadlineGroup(
            new WaitCommand(8),
            new ShootWithLimelight(),
            new MoveBallsToShooter()
        )
    );







    // public static SequentialCommandGroup turn = new SequentialCommandGroup(
    //     new RotateToAngle(15)
    // );

    public static SequentialCommandGroup leftToUpAgainstGoal = new SequentialCommandGroup(
        new ParallelRaceGroup(
            new HSSwerveDriveController(Trajectories.chezy_leftInitiationToScoringZone, Rotation2d.fromDegrees(0)),
            new ShooterVelocityManual(65)
        ),
        new ParallelDeadlineGroup(
            new WaitCommand(5),
            new ShootWithHighHood(),
            new MoveBallsToShooter()
        )
    );

    // public static SequentialCommandGroup rotateAndShoot = new SequentialCommandGroup(

    //     new ParallelDeadlineGroup(
    //         new WaitCommand(1),
    //         new RotateInPlace()
    //     ),

    //     new ParallelDeadlineGroup(
    //         new WaitCommand(8),
    //         new ShootWithHighHood(),
    //         new MoveBallsToShooter()
    //     )
    // );

    // public static SequentialCommandGroup justShooting = new SequentialCommandGroup(

    //     new ParallelDeadlineGroup(
    //         new WaitCommand(8),
    //         new ShootWithHighHood(),
    //         new MoveBallsToShooter()
    //     )
    // );

    public static SequentialCommandGroup centerToUpAgainstGoal = new SequentialCommandGroup(
        new ParallelRaceGroup(
            new HSSwerveDriveController(Trajectories.chezy_centerInitiationToScoringZone, Rotation2d.fromDegrees(0)),
            new Rev(70)
        ),
        new ParallelDeadlineGroup(
            new WaitCommand(8),
            new ShootWithHighHood(),
            new MoveBallsToShooter()
        )
    );

    public static SequentialCommandGroup rightToUpAgainstGoal = new SequentialCommandGroup(
        new ParallelRaceGroup(
            new HSSwerveDriveController(Trajectories.chezy_rightInitiationToScoringZone, Rotation2d.fromDegrees(0)),
            new ShooterVelocityManual(65)
        ),
        new ParallelDeadlineGroup(
            new WaitCommand(5),
            new ShootWithHighHood(),
            new MoveBallsToShooter()
        )
    );

    // public static SequentialCommandGroup throughTrench = new SequentialCommandGroup(

    //     new ParallelDeadlineGroup(
    //         new WaitCommand(5),
    //         new ShootWithLimelight(),
    //         new MoveBallsToShooter()
    //     ),
    //     new InstantCommand(() -> {
    //         Intake.getInstance().invertSolenoid();
    //     }),
    //     //new HSSwerveDriveController(Trajectories.chezy_setUpLeftTrench, Rotation2d.fromDegrees(0)),

    //     new ParallelRaceGroup(
    //         new HSSwerveDriveController(Trajectories.chezy_moveThroughTrench, Rotation2d.fromDegrees(0)),
    //         new IntakeAutonControlForward(0.5)
    //     ),
        
    //     new HSSwerveDriveController(Trajectories.chezy_moveBackThroughTrench, Rotation2d.fromDegrees(0)),
    //     new ParallelRaceGroup(
    //         new HSSwerveDriveController(Trajectories.chezy_shootTrenchBalls, Rotation2d.fromDegrees(0)),
    //         new ShooterVelocityManual(65)
    //     ),
    //     new ParallelDeadlineGroup(
    //         new WaitCommand(5),
    //         new ShootWithLimelight(),
    //         new MoveBallsToShooter()
    //     )

    // );
    public static SequentialCommandGroup getAutoCommand(){
        String dash = SmartDashboard.getString("autoPath", "steal");
        if(dash.equals("left")){
            return leftToUpAgainstGoal;
        }
        else if(dash.equals("right")){
            return rightToUpAgainstGoal;
        }
        else if(dash.equals("center")){
            return centerToUpAgainstGoal;
        }
        return stealBallsFromTrench;
    }
    public static SequentialCommandGroup autonCommand = stealBallsFromTrench;
}
