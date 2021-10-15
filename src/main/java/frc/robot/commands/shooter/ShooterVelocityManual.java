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
        this.velocity = velocity;
        SmartDashboard.putNumber("desired shooter vel", 40);
        SmartDashboard.putNumber("Shooter V error", Shooter.getInstance().getRotation().getClosedLoopError());
        SmartDashboard.putNumber("Shooter % output", Shooter.getInstance().getRotation().getMotorOutputPercent());  

    }

    public void initialize() {
        // Indexer.getInstance().getSolenoid().set(Value.kReverse);
    }

    @Override
    public void execute(){
        SmartDashboard.putNumber("Shooter V error", Shooter.getInstance().getRotation().getClosedLoopError());
        SmartDashboard.putNumber("Shooter % output", Shooter.getInstance().getRotation().getMotorOutputPercent());
        Shooter.getInstance().setVelocity(SmartDashboard.getNumber("desired shooter vel", 220));

    }

    @Override
    public void end(boolean a){
        // Indexer.getInstance().getSolenoid().set(Value.kForward);

        Shooter.getInstance().setVelocity(0);;

    }
    
}
