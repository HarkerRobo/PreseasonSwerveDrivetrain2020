package frc.robot.commands.climber;

import harkerrobolib.commands.IndefiniteCommand;
import frc.robot.OI;
import frc.robot.subsystems.Climber;

public class ClimberManual extends IndefiniteCommand
{
    private static final double OUTPUT_MAGNITUDE = 0.7;

    public ClimberManual() {
        addRequirements(Climber.getInstance());
    }

    @Override
    public void execute(){
        if(OI.getInstance().getDriverGamepad().getUpDPadButton().get()){
            Climber.getInstance().setPercentOutput(OUTPUT_MAGNITUDE);
        }
        else if(OI.getInstance().getDriverGamepad().getDownDPadButton().get()){
            Climber.getInstance().setPercentOutput(-OUTPUT_MAGNITUDE);
        }
        else{
            Climber.getInstance().setPercentOutput(0);
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        Climber.getInstance().setPercentOutput(0);
    }
}