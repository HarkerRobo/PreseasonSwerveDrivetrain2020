package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.MathUtil;

import frc.robot.util.Vector;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import frc.robot.OI;

public class SwerveManual extends IndefiniteCommand {
    public SwerveManual() {
        addRequirements(Drivetrain.getInstance());
    }

    @Override
    public void execute() {
        double angularVelocity = MathUtil.mapJoystickOutput(OI.getInstance().getOperatorGamepad().getLeftX(), OI.DEADBAND);
        double translationx = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftX(), OI.DEADBAND);
        double translationy = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftY(), OI.DEADBAND);


        angularVelocity*=Drivetrain.MAX_SPEED;

        translationx*=Drivetrain.MAX_SPEED;
        translationy*=Drivetrain.MAX_SPEED;

        // double rotation =
        // MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getRightX(),
        // OI.DEADBAND);

        ChassisSpeeds chassis=new ChassisSpeeds(translationx, translationy, angularVelocity);

        Drivetrain.getInstance().setAngleAndDriveVelocity(Drivetrain.getInstance().getSwerveDriveKinematics().toSwerveModuleStates(chassis));
    }
}
