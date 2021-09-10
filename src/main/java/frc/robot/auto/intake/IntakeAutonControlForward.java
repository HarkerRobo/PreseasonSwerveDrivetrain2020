package frc.robot.auto.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.intake.IntakeManualPID;
import frc.robot.subsystems.Intake;
import harkerrobolib.commands.IndefiniteCommand;

public class IntakeAutonControlForward extends IndefiniteCommand {
    private Command runForward;

    IntakeAutonControlForward(double velocity) {       
        runForward = new IntakeManualPID(velocity, false); 
        addRequirements(Intake.getInstance());
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        runForward.execute();
    }

    @Override
    public void end(boolean a) {
        runForward.end(true);
    }
}