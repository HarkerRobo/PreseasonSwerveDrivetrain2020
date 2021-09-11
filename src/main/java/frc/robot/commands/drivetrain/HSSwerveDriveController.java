package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.Drivetrain;

public class HSSwerveDriveController extends SwerveControllerCommand {
    private static final double kP=2;
    private static final double kI=0;
    private static final double kD=0;

    private static final double THETA_kP=3;
    private static final double THETA_kI=0;
    private static final double THETA_kD=0;

    private static PIDController xController = new PIDController(kP, kI, kD);
    private static PIDController yController = new PIDController(kP, kI, kD);
    private static ProfiledPIDController thetaController = new ProfiledPIDController(THETA_kP, THETA_kI, THETA_kD, new Constraints(Drivetrain.AUTO_MAX_SPEED, Drivetrain.AUTO_MAX_SPEED_ACCELERATION));
    
    private Trajectory trajectory;

  
    public HSSwerveDriveController(Trajectory trajectory) {
        super(trajectory, Drivetrain.getInstance().getOdometry()::getPoseMeters, 
        Drivetrain.getInstance().getKinematics(), 
        xController, 
        yController, 
        thetaController,
        Drivetrain.getInstance()::setAngleAndDriveVelocity,
        Drivetrain.getInstance());
        this.trajectory=trajectory;
    }
    
    @Override
    public void initialize() {
        super.initialize();
        Drivetrain.getInstance().getOdometry().resetPosition(trajectory.getInitialPose(), Rotation2d.fromDegrees(Drivetrain.getInstance().getPigeon().getFusedHeading()));
    }

    @Override
    public void execute() {
        super.execute();
        SmartDashboard.putNumber("mp x error", xController.getPositionError());
        SmartDashboard.putNumber("mp y error", yController.getPositionError());
        SmartDashboard.putNumber("mp theta error", thetaController.getPositionError());
    }
}