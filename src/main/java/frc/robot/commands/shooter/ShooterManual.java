package frc.robot.commands.shooter;

import frc.robot.subsystems.Shooter;
import harkerrobolib.commands.IndefiniteCommand;


public class ShooterManual extends IndefiniteCommand {
    private static final double OUTPUT_MULTIPLIER= 0.2;
    public ShooterManual() {
        addRequirements(Shooter.getInstance());
    }

    @Override
    public void execute(){
        double output = 0.7;
        // output*=OUTPUT_MULTIPLIER;
        Shooter.getInstance().setPercentOutput(output);

    }

    @Override
    public void end(boolean a){
        double output = 0;
        // output*=OUTPUT_MULTIPLIER;
        Shooter.getInstance().setPercentOutput(output);

    }
}
