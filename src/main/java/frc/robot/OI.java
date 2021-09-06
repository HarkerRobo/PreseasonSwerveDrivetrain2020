package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.auto.Trajectories;
import frc.robot.commands.drivetrain.HSSwerveDriveController;
import frc.robot.commands.intake.MoveBallsToShooter;
import frc.robot.commands.shooter.ShooterManual;
import frc.robot.commands.shooter.ShooterVelocityManual;
import frc.robot.commands.spine.Jumble;
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
        operatorGamepad.getButtonA().whenPressed(new InstantCommand(() -> {
            Intake.getInstance().invertSolenoid();
        }, Intake.getInstance()));

        operatorGamepad.getButtonB().whenPressed(new InstantCommand(() -> {
            Shooter.getInstance().invertSolenoid();
        }, Shooter.getInstance()));
        
        // driverGamepad.getButtonBumperRight().whilePressed(new ParallelCommandGroup(
        //     new MoveBallsToShooter(), new ShooterManual()
        // ));

        driverGamepad.getButtonY().whilePressed(new ParallelCommandGroup(
            new Jumble()
        ));
        operatorGamepad.getButtonY().whilePressed(new ParallelCommandGroup(
            new ShooterVelocityManual()
        ));
        driverGamepad.getUpDPadButton().whenPressed(new HSSwerveDriveController(Trajectories.sShaped));
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
