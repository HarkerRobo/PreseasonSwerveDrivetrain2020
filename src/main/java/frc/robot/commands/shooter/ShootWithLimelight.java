package frc.robot.commands.shooter;

import harkerrobolib.commands.IndefiniteCommand;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Limelight;

public class ShootWithLimelight extends IndefiniteCommand {

    public ShootWithLimelight() {
        addRequirements(Shooter.getInstance());
        Indexer.getInstance().getSolenoid().set(Value.kReverse);
    }

    @Override
    public void execute(){
        SmartDashboard.putNumber("Shooter V error", Shooter.getInstance().getRotation().getClosedLoopError());
        SmartDashboard.putNumber("Shooter % output", Shooter.getInstance().getRotation().getMotorOutputPercent());
        double distance = (Shooter.POWER_PORT_HEIGHT-Limelight.LIMELIGHT_HEIGHT) / Math.tan(Math.toRadians(Limelight.LIMELIGHT_ANGLE+Limelight.getTy()));
        double limelightDistance = (Shooter.POWER_PORT_HEIGHT-Limelight.LIMELIGHT_HEIGHT) / Math.tan(Math.toRadians(Limelight.LIMELIGHT_ANGLE+Limelight.getTy()));
        double velocity = 55.3576 + (1.3285*limelightDistance) + (0.00182129 * Math.pow(limelightDistance,2));
        double hoodAngle = 0.376581 + (0.00635765 * limelightDistance) + (-0.00001741 * Math.pow(limelightDistance, 2));
        Shooter.getInstance().setVelocity(velocity);
        Shooter.getInstance().setHoodAngle(hoodAngle);
    }

    @Override
    public void end(boolean a){

        Shooter.getInstance().setPercentOutput(0);;

    }
    
}



