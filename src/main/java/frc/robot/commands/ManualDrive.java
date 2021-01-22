package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import harkerrobolib.commands.IndefiniteCommand;

public class ManualDrive extends IndefiniteCommand{
    public ManualDrive(){
        addRequirements(Drivetrain.getInstance());
    }

    @Override
    public void execute(){
        
    }
}
