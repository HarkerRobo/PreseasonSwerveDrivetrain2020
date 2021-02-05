package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.MathUtil;

import frc.robot.util.Vector;



import frc.robot.OI;


public class ManualDrive extends IndefiniteCommand{
    public ManualDrive(){
        addRequirements(Drivetrain.getInstance());
    }

    @Override
    public void execute(){
        double translationx=MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftX(), OI.DEADBAND);
        double translationy=MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftY(), OI.DEADBAND);

        double rotation=MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getRightX(), OI.DEADBAND);

        Drivetrain.getInstance().setPercentOutput(new Vector(translationx, translationy));
    }
}
