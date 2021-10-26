package frc.robot.commands.shooter;

import harkerrobolib.commands.IndefiniteCommand;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Limelight;

public class ShootWithLimelight extends IndefiniteCommand {

    public ShootWithLimelight() {
        addRequirements(Shooter.getInstance());
        Indexer.getInstance().getSolenoid().set(Value.kReverse);
    }

    @Override
    public void execute(){
        SmartDashboard.putNumber("Shooter V error", Shooter.getInstance().getRotation().getClosedLoopError());
        SmartDashboard.putNumber("Shooter % output", Shooter.getInstance().getRotation().getMotorOutputPercent());
        double limelightDistance = Shooter.getInstance().getDistance();
        double velocity = 55.3576 + (1.3285*limelightDistance) + (0.00182129 * limelightDistance * limelightDistance) + Shooter.getInstance().velAdjustment;
        velocity = Math.min(Math.max(velocity, 0), Shooter.MAX_VEL);
        if(Limelight.isTargetVisible()) Shooter.getInstance().setVelocity(velocity);
        else Shooter.getInstance().setVelocity(80);
        
    }

    @Override
    public void end(boolean interrupted){
        Shooter.getInstance().setPercentOutput(0);;
    }
    
}



