package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.MathUtil;
import frc.robot.OI;


public class ManualDrive extends IndefiniteCommand{
    public ManualDrive(){
        addRequirements(Drivetrain.getInstance());
    }

    @Override
    public void execute(){
        double rotation=MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftX(), OI.DEADBAND);
        
        double translation=MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftY(), OI.DEADBAND);

        Drivetrain.getInstance().setPercentOutput(rotation, translation);
    }
}
