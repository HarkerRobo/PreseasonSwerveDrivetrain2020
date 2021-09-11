package frc.robot.commands.climber;

import harkerrobolib.commands.IndefiniteCommand;
import frc.robot.subsystems.Climber;

public class ClimberManual extends IndefiniteCommand
{
    public double OUTPUT_MAGNITUDE = 0.5;

    public ClimberManual() {
        addRequirements(Climber.getInstance());
    }

    @Override
    public void execute(){
        double output = 0.7;
        output *= OUTPUT_MAGNITUDE;
        Climber.getInstance().setPercentOutput(output);
    }

    @Override
    public void end(boolean interrupted)
    {
        Climber.getInstance().setPercentOutput(0);
    }
}