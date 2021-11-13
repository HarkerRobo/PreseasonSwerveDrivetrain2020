package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import harkerrobolib.wrappers.HSFalcon;

public class Climber extends SubsystemBase {
    private static Climber instance;
    private HSFalcon master;
    private HSFalcon follower;
    private static final double VOLTAGE_COMP = 10;
    private static final double ANGLE_CURRENT_CONTINUOUS = 70;
    private static final double ANGLE_CURRENT_PEAK = 70;
    private static final double ANGLE_CURRENT_PEAK_DUR = 100;

    private static final boolean MASTER_INVERTED = true;
    private static final boolean FOLLOWER_INVERTED = false;

    private Climber(){
        master=new HSFalcon(RobotMap.CLIMBER_MASTER);
        follower=new HSFalcon(RobotMap.CLIMBER_FOLLOWER);
        climberInit();
    }

    public void climberInit(){
        master.configFactoryDefault();
        follower.configFactoryDefault();
        master.setNeutralMode(NeutralMode.Brake);
        follower.setNeutralMode(NeutralMode.Brake);
		master.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, ANGLE_CURRENT_CONTINUOUS, ANGLE_CURRENT_PEAK, ANGLE_CURRENT_PEAK_DUR));
		// master.configVoltageCompSaturation(VOLTAGE_COMP);
        // master.configForwardSoftLimitEnable(true);
        // master.configReverseSoftLimitEnable(true);
        follower.follow(master);

        master.setInverted(MASTER_INVERTED);
        follower.setInverted(FOLLOWER_INVERTED);


    }
    
    public void setPercentOutput(double output){
        master.set(ControlMode.PercentOutput, output);
    }

    public static Climber getInstance() {
        if (instance == null) {
           instance = new Climber();
        }
        return instance;
    }
}
