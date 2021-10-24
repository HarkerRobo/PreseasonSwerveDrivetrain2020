package frc.robot.commands.shooter;

import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.Conversions;
import harkerrobolib.util.Conversions.SpeedUnit;
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

    }

    public void initialize() {
        // Indexer.getInstance().getSolenoid().set(Value.kReverse);
    }

    @Override
    public void execute(){
        Shooter.getInstance().setVelocity(velocity);

    }

    @Override
    public void end(boolean a){
        // Indexer.getInstance().getSolenoid().set(Value.kForward);

        Shooter.getInstance().setPercentOutput(0);

    }
    
}
