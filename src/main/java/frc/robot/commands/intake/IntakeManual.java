package frc.robot.commands.intake;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.MathUtil;

import frc.robot.util.Vector;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;

public class IntakeManual extends IndefiniteCommand {
    private static final double OUTPUT_MULTIPLIER= 0.2;
    public IntakeManual() {
        addRequirements(Intake.getInstance());
    }

    @Override
    public void execute(){
        double output = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getRightTrigger(), OI.DEADBAND);
        output*=OUTPUT_MULTIPLIER;
        Intake.getInstance().setPercentOutput(output);

    }
}
