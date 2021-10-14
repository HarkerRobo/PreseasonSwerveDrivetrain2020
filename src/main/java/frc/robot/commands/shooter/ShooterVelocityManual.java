package frc.robot.commands.shooter;

import harkerrobolib.commands.IndefiniteCommand;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Limelight;

public class ShooterVelocityManual extends IndefiniteCommand {
private double velocity;

    public ShooterVelocityManual(double velocity){
        addRequirements(Shooter.getInstance());
        Indexer.getInstance().getSolenoid().set(Value.kReverse);
        this.velocity = velocity;

    }

    @Override
    public void execute(){
        SmartDashboard.putNumber("Shooter V error", Shooter.getInstance().getRotation().getClosedLoopError());
        SmartDashboard.putNumber("Shooter % output", Shooter.getInstance().getRotation().getMotorOutputPercent());

        Shooter.getInstance().setVelocity(velocity);

    }

    @Override
    public void end(boolean a){

        Shooter.getInstance().setVelocity(0);;

    }
    
}
