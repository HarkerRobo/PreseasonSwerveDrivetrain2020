package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.MathUtil;

import frc.robot.util.Vector;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;

public class SwerveManual extends IndefiniteCommand {
    private static final double OUTPUT_MULTIPLIER= 0.2;

    public SwerveManual() {
        addRequirements(Drivetrain.getInstance());
    }

    @Override
    public void execute() {
        double angularVelocity = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getRightX(), OI.DEADBAND);
        double translationx = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftX(), OI.DEADBAND);
        double translationy = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftY(), OI.DEADBAND);


        angularVelocity *= Drivetrain.MAX_ANGULAR_VEL;

        translationx *= Drivetrain.MAX_DRIVE_VEL * OUTPUT_MULTIPLIER;
        translationy *= Drivetrain.MAX_DRIVE_VEL * OUTPUT_MULTIPLIER;

       // System.out.println(translationx + " " + translationy);
        // double rotation =
        // MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getRightX(),
        // OI.DEADBAND);

        ChassisSpeeds chassis = new ChassisSpeeds(translationx, translationy, -angularVelocity);
        
        Drivetrain.getInstance().setAngleAndDriveVelocity(Drivetrain.getInstance().getSwerveDriveKinematics().toSwerveModuleStates(chassis), false);
    }
}
