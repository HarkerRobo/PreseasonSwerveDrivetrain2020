package frc.robot.commands.drivetrain;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.MathUtil;
import frc.robot.util.Limelight;
import frc.robot.util.Vector;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.RobotMap;

public class RotateToAngle extends IndefiniteCommand {
    private static final double kP=0.03;
    private static final double kI=0.0;//00002;
    private static final double kD=0.0;//02;
    private static final double TX_SETPOINT=0;
    private static final double I_ZONE = 0;
    private double angle=0;


    private PIDController pid;
    public RotateToAngle(double angle) {
        addRequirements(Drivetrain.getInstance());
        pid = new PIDController(kP, kI, kD);
        pid.setIntegratorRange(-I_ZONE, I_ZONE);
        this.angle=angle;
    }

    @Override
    public void execute() {
        double angVel = -pid.calculate(Drivetrain.getInstance().getPigeon().getFusedHeading(), -angle);

        angVel *= Drivetrain.MAX_ANGULAR_VEL;


        ChassisSpeeds chassis = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, -angVel, new Rotation2d(Math.toRadians(Drivetrain.getInstance().getPigeon().getFusedHeading())));

        Drivetrain.getInstance().setAngleAndDriveVelocity(Drivetrain.getInstance().getKinematics().toSwerveModuleStates(chassis));
    }

    @Override
    public void end(boolean interrupted) {
        Drivetrain.getInstance().setPercentOutput(new Vector(0, 0));
    }
}
