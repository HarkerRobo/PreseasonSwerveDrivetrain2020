package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import harkerrobolib.commands.CallMethodCommand;
import harkerrobolib.wrappers.HSGamepad;
import harkerrobolib.wrappers.XboxGamepad;

public class OI {
    private static OI oi;
    private HSGamepad driverGamepad;
    private HSGamepad operatorGamepad;
    public static final double DEADBAND = 0.1;
    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;
    
    private OI() {
        driverGamepad = new XboxGamepad(DRIVER_PORT);
        operatorGamepad = new XboxGamepad(OPERATOR_PORT);

        initBindings();
    }

    private void initBindings() {
        driverGamepad.getButtonA().whenPressed(new InstantCommand(() -> {
            Intake.getInstance().invertSolenoid();
        }, Intake.getInstance()));

        driverGamepad.getButtonB().whenPressed(new InstantCommand(() -> {
            Shooter.getInstance().invertSolenoid();
        }, Shooter.getInstance()));

    }
    
    public HSGamepad getDriverGamepad(){
        return driverGamepad;
    }
    public HSGamepad getOperatorGamepad(){
        return operatorGamepad;
    }

    public static OI getInstance(){
        if(oi==null)
            oi=new OI();
        return oi;
    }
}
