package frc.robot.commands.shooter;

import harkerrobolib.commands.IndefiniteCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Limelight;

public class ShooterVelocityManual extends IndefiniteCommand {
public final double VELOCITY=110;

    public ShooterVelocityManual(){
        addRequirements(Shooter.getInstance());
    }

    @Override
    public void execute(){
        SmartDashboard.putNumber("Shooter V error", Shooter.getInstance().getRotation().getClosedLoopError());
        SmartDashboard.putNumber("Shooter % output", Shooter.getInstance().getRotation().getMotorOutputPercent());
        double distance = (Shooter.POWER_PORT_HEIGHT-Limelight.LIMELIGHT_HEIGHT) / Math.tan(Math.toRadians(18+Limelight.getTy()));

        Shooter.getInstance().setVelocity(VELOCITY*Shooter.getInstance().GEAR_RATIO);

    }

    @Override
    public void end(boolean a){

        Shooter.getInstance().setVelocity(0);;

    }
    
}
