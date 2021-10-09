package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import harkerrobolib.commands.IndefiniteCommand;

public class MoveBallsToShooter extends IndefiniteCommand{
    private static final double AGITATOR_MAX_SPEED = 0.8;
    private static final double LINEAR_MAX_SPEED = 0.8;
    private static final long MIN_DELAY = 2000;
    private long commandTime;

    public MoveBallsToShooter(){
        addRequirements(Indexer.getInstance());
        addRequirements(Intake.getInstance());
    }
    
    @Override
    public void initialize() {
        commandTime = System.currentTimeMillis();
    }

    @Override
    public void execute(){
        long currentTime = System.currentTimeMillis();
        if(currentTime - commandTime > MIN_DELAY) {
            Indexer.getInstance().getSolenoid().set(Indexer.BLOCKER_OPEN);
            Indexer.getInstance().setLinearPercentOutput(LINEAR_MAX_SPEED);
            
    
            if (currentTime % 1000 < 500) {
                Indexer.getInstance().setAgitatorPercentOutput(AGITATOR_MAX_SPEED);
            } else {
                Indexer.getInstance().setAgitatorPercentOutput(-AGITATOR_MAX_SPEED);
            }
    
        }
    }

    @Override
    public void end(boolean a){
        Indexer.getInstance().getSolenoid().set(Indexer.BLOCKER_CLOSED);
        Indexer.getInstance().setLinearPercentOutput(0); 
        Indexer.getInstance().setAgitatorPercentOutput(0);

    }
}
