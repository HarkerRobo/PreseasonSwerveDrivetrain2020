package frc.robot.commands.shooter;

import harkerrobolib.commands.IndefiniteCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Shooter;

public class ShooterVelocityManual extends IndefiniteCommand {
public final double VELOCITY=110;

    public ShooterVelocityManual(){
        addRequirements(Shooter.getInstance());
    }

    @Override
    public void execute(){
        SmartDashboard.putNumber("Shooter V error", Shooter.getInstance().getRotation().getClosedLoopError());
        SmartDashboard.putNumber("Shooter % output", Shooter.getInstance().getRotation().getMotorOutputPercent());

        Shooter.getInstance().setVelocity(VELOCITY*Shooter.getInstance().GEAR_RATIO);

    }

    @Override
    public void end(boolean a){

        Shooter.getInstance().setVelocity(0);;

    }
    
}
