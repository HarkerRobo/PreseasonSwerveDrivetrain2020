package frc.robot.commands.drivetrain;

import frc.robot.subsystems.Drivetrain;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.MathUtil;
import frc.robot.util.Limelight;
import frc.robot.util.Vector;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;

public class SwerveManual extends IndefiniteCommand {
    private static final double OUTPUT_MULTIPLIER= 0.2;
    private static final double kP=0.03;
    private static final double kI=0.0002;
    private static final double kD=0.008;
    private static final double TX_SETPOINT=0;
    private static final double I_ZONE = 1;
    private PIDController pid;
    public SwerveManual() {
        addRequirements(Drivetrain.getInstance());
        pid = new PIDController(kP, kI, kD);
        pid.setIntegratorRange(-I_ZONE, I_ZONE);
    }

    @Override
    public void execute() {
        double angularVelocity = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getRightX(), OI.DEADBAND);
        double translationx = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftX(), OI.DEADBAND);
        double translationy = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftY(), OI.DEADBAND);
        double chasisMagnitude=Math.sqrt(translationx*translationx + translationy*translationy);

        if(chasisMagnitude<(Drivetrain.MIN_OUTPUT)){
            translationx=0;
            translationy=0;
            if(Math.abs(angularVelocity)<(Drivetrain.MIN_OUTPUT)){
            angularVelocity=0.01;}
        }
    

        angularVelocity *= Drivetrain.MAX_ANGULAR_VEL;
        if(OI.getInstance().getDriverGamepad().getButtonBumperLeftState()){
            angularVelocity = -pid.calculate(Limelight.getTx(), TX_SETPOINT);
        }
        SmartDashboard.putNumber("limelight tx", Limelight.getTx());
        SmartDashboard.putNumber("limelight ang vel", angularVelocity);
        translationx *= Drivetrain.MAX_DRIVE_VEL * OUTPUT_MULTIPLIER;
        translationy *= Drivetrain.MAX_DRIVE_VEL * OUTPUT_MULTIPLIER;

       // System.out.println(translationx + " " + translationy);
        // double rotation =
        // MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getRightX(),
        // OI.DEADBAND);

        ChassisSpeeds chassis = ChassisSpeeds.fromFieldRelativeSpeeds(translationx, translationy, -angularVelocity, new Rotation2d(Math.toRadians(Drivetrain.getInstance().getPigeon().getFusedHeading())));

        Drivetrain.getInstance().setAngleAndDriveVelocity(Drivetrain.getInstance().getSwerveDriveKinematics().toSwerveModuleStates(chassis), false);
    }
}
