package frc.robot.commands.shooter;

import harkerrobolib.commands.IndefiniteCommand;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Limelight;

public class ShootWithHighHood extends IndefiniteCommand {

    public ShootWithHighHood() {
        addRequirements(Shooter.getInstance());
    }

    @Override
    public void execute(){
        Shooter.getInstance().setVelocity(150 + Shooter.getInstance().highVelAdjustment);
        Shooter.getInstance().setHoodAngle(0.27);
    }

    @Override
    public void end(boolean interrupted){
        Shooter.getInstance().setPercentOutput(0);
    }
    
}



