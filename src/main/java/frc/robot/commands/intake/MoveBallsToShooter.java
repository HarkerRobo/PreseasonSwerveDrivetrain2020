package frc.robot.commands.intake;

import frc.robot.subsystems.Indexer;
import harkerrobolib.commands.IndefiniteCommand;

public class MoveBallsToShooter extends IndefiniteCommand{
    private static final double AGITATOR_MAX_SPEED = 0.8;
    private static final double LINEAR_MAX_SPEED = 0.4;
    private long commandTime;

    public MoveBallsToShooter(){
        addRequirements(Indexer.getInstance());
    }
    
    @Override
    public void initialize() {
        commandTime = System.currentTimeMillis();
    }

    @Override
    public void execute(){
        Indexer.getInstance().getSolenoid().set(Indexer.BLOCKER_OPEN);
        Indexer.getInstance().setLinearPercentOutput(LINEAR_MAX_SPEED);
        if (commandTime % 1000 < 500) {
            Indexer.getInstance().setAgitatorPercentOutput(AGITATOR_MAX_SPEED);
        } else {
            Indexer.getInstance().setAgitatorPercentOutput(-AGITATOR_MAX_SPEED);
        }
    }

    @Override
    public void end(boolean a){
        Indexer.getInstance().getSolenoid().set(Indexer.BLOCKER_CLOSED);
        Indexer.getInstance().setLinearPercentOutput(0); 
        Indexer.getInstance().setAgitatorPercentOutput(0);

    }
}
