package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.MathUtil;

import frc.robot.util.Vector;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;

public class ShooterManual extends IndefiniteCommand {
    private static final double OUTPUT_MULTIPLIER= 0.2;
    public ShooterManual() {
        addRequirements(Shooter.getInstance());
    }

    @Override
    public void execute(){
        double output = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftTrigger(), OI.DEADBAND);
        output*=OUTPUT_MULTIPLIER;
        Shooter.getInstance().setPercentOutput(output);

    }
}
