package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import frc.robot.RobotMap;
import frc.robot.util.SwerveModule;
import harkerrobolib.wrappers.HSTalon;

public class Drivetrain extends SubsystemBase {
    private static  Drivetrain drivetrain;


    private SwerveModule topLeft;
    private SwerveModule topRight;
    private SwerveModule bottomLeft;
    private SwerveModule bottomRight;

    private boolean TOP_LEFT_ROTATION_SENSOR_PHASE=false;
    private int[] TOP_LEFT_DRIVE_IDS= new int[]{RobotMap.ROTATION_IDS[0], RobotMap.TRANSLATION_IDS[1]};

    private Drivetrain() {
        topLeft=new SwerveModule(TOP_LEFT_ROTATION_SENSOR_PHASE, TOP_LEFT_ROTATION_SENSOR_PHASE, TOP_LEFT_DRIVE_IDS);
    }

    public static Drivetrain getInstance() {
        if (drivetrain == null) {
            drivetrain = new Drivetrain();
        }
        return drivetrain;
    }
}