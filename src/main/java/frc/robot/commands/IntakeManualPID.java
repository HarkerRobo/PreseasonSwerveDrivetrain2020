package frc.robot.commands;

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

public class IntakeManualPID extends IndefiniteCommand {
    private static final double MAX_VELOCITY= 15;
    public IntakeManualPID() {
        addRequirements(Intake.getInstance());
    }

    @Override
    public void execute(){
        double magnitude = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getRightTrigger(), OI.DEADBAND);
        magnitude *= MAX_VELOCITY;
        Intake.getInstance().setVelocity(magnitude);
        SmartDashboard.putNumber("desired velocity", magnitude);
        SmartDashboard.putNumber("intake current velocity", Intake.getInstance().getRotation().getSelectedSensorVelocity());
        SmartDashboard.putNumber("velocity error", Intake.getInstance().getRotation().getClosedLoopError());
        SmartDashboard.putNumber("intake percent  output", Intake.getInstance().getRotation().getMotorOutputPercent());;

    }
}
