package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.Drivetrain;

public class HSSwerveDriveController extends SwerveControllerCommand {
    private static final double kP=6;
    private static final double kI=0;
    private static final double kD=6;

    private static final double THETA_kP=7;
    private static final double THETA_kI=0;
    private static final double THETA_kD=4;

    private static PIDController xController = new PIDController(kP, kI, kD);
    private static PIDController yController = new PIDController(kP, kI, kD);
    private static ProfiledPIDController thetaController = new ProfiledPIDController(THETA_kP, THETA_kI, THETA_kD, new Constraints(2 * Math.PI, 3 * Math.PI));
    
    private Trajectory trajectory;

  
    public HSSwerveDriveController(Trajectory trajectory, Rotation2d heading) {
        super(trajectory, Drivetrain.getInstance().getOdometry()::getPoseMeters, 
        Drivetrain.getInstance().getKinematics(), 
        xController, 
        yController, 
        thetaController,
        () -> heading,
        Drivetrain.getInstance()::setAngleAndDriveVelocity,
        Drivetrain.getInstance());
        this.trajectory=trajectory;
    }
    
    @Override
    public void initialize() {
        super.initialize();
        Drivetrain.getInstance().getOdometry().resetPosition(new Pose2d(trajectory.getInitialPose().getTranslation(),Rotation2d.fromDegrees(Drivetrain.getInstance().getPigeon().getFusedHeading())),
            Rotation2d.fromDegrees(Drivetrain.getInstance().getPigeon().getFusedHeading()));
    }

    @Override
    public void execute() {
        super.execute();
        SmartDashboard.putNumber("mp x error", xController.getPositionError());
        SmartDashboard.putNumber("mp y error", yController.getPositionError());
        SmartDashboard.putNumber("mp theta error", thetaController.getPositionError());
    }
}