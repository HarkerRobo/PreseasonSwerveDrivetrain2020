package frc.robot.commands.shooter;

import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.Conversions;
import harkerrobolib.util.Conversions.SpeedUnit;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Limelight;

public class ShooterRevUp extends IndefiniteCommand {
    public ShooterRevUp(){
        addRequirements(Shooter.getInstance());
    }

    public void initialize() {
        // Indexer.getInstance().getSolenoid().set(Value.kReverse);
    }

    @Override
    public void execute(){
        Shooter.getInstance().setPercentOutput(0.5);

    }

    @Override
    public void end(boolean a){
        // Indexer.getInstance().getSolenoid().set(Value.kForward);

        Shooter.getInstance().setPercentOutput(0);

    }
    
}
