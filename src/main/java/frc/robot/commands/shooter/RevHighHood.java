package frc.robot.commands.shooter;

import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.Conversions;
import harkerrobolib.util.Conversions.SpeedUnit;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Limelight;

public class RevHighHood extends IndefiniteCommand {
    private long commandTime;
    private static final long DELAY = 100;
    private static final double AGITATOR_MAX_SPEED = 0.8;
    private static final double LINEAR_MAX_SPEED = 0.5;

    public RevHighHood(){
        addRequirements(Shooter.getInstance());
    }

    @Override
    public void initialize() {
        commandTime = System.currentTimeMillis();
        
    }

    @Override
    public void execute(){
        
        long currentTime = System.currentTimeMillis();

        if(currentTime - commandTime < DELAY){
            Indexer.getInstance().setLinearPercentOutput(-LINEAR_MAX_SPEED);
            Indexer.getInstance().setAgitatorPercentOutput(AGITATOR_MAX_SPEED);
        }
        else{
            Indexer.getInstance().setLinearPercentOutput(0);
            Indexer.getInstance().setAgitatorPercentOutput(0);
            Shooter.getInstance().setVelocity(120);
            Shooter.getInstance().setHoodAngle(0.27);
        }
    }

    @Override
    public void end(boolean a){
        Indexer.getInstance().setLinearPercentOutput(0);
        Indexer.getInstance().setAgitatorPercentOutput(0);
        Shooter.getInstance().setPercentOutput(0);

        
    }
    
}
