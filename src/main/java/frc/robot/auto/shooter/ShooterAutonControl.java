package frc.robot.auto.shooter;
import harkerrobolib.commands.IndefiniteCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Limelight;

public class ShooterAutonControl extends IndefiniteCommand {
    public double velocity;

    public ShooterAutonControl(double velocity){
        this.velocity = velocity;
        addRequirements(Shooter.getInstance());
    }

    @Override
    public void execute(){
        Shooter.getInstance().setVelocity(velocity * Shooter.getInstance().GEAR_RATIO);
    }

    @Override
    public void end(boolean a){
        Shooter.getInstance().setVelocity(0);
    }
}
