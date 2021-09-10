package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.fasterxml.jackson.core.format.MatchStrength;

import frc.robot.RobotMap;
import harkerrobolib.wrappers.HSFalcon;

public class Climber {
    private static Climber instance;
    private HSFalcon master;
    private HSFalcon follower;
    private static final double VOLTAGE_COMP = 10;
    private static final double ANGLE_CURRENT_CONTINUOUS = 60;
    private static final double ANGLE_CURRENT_PEAK = 70;
    private static final double ANGLE_CURRENT_PEAK_DUR = 100;

    private Climber(){
        master=new HSFalcon(RobotMap.CLIMBER_MASTER);
        follower=new HSFalcon(RobotMap.CLIMBER_FOLLOWER);
    }
    public void climberInit(){
        master.configFactoryDefault();
        master.setNeutralMode(NeutralMode.Brake);
		master.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, ANGLE_CURRENT_CONTINUOUS, ANGLE_CURRENT_PEAK, ANGLE_CURRENT_PEAK_DUR));
		master.configVoltageCompSaturation(VOLTAGE_COMP);
        master.configForwardSoftLimitEnable(true);
        master.configReverseSoftLimitEnable(true);
        follower.follow(master);
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
