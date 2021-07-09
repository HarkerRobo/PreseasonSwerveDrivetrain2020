package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import harkerrobolib.wrappers.HSTalon;

public class Intake extends SubsystemBase {
    private static Intake instance;
    private HSTalon rotation;
    private static final double ANGLE_P = 1.1;
	private static final double ANGLE_I = 0;
    private static final double ANGLE_D = 11;
    private static final double VOLTAGE_COMP = 10;
    private static final double ANGLE_CURRENT_CONTINUOUS = 15;
    private static final double ANGLE_CURRENT_PEAK = 20;
    private static final double ANGLE_CURRENT_PEAK_DUR = 0.02;

    public Intake() {
        rotation=new HSTalon(RobotMap.INTAKE);
    }

    public void intakeInit(){
        rotation.configFactoryDefault();
        rotation.setNeutralMode(NeutralMode.Brake);
		rotation.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, ANGLE_CURRENT_CONTINUOUS, ANGLE_CURRENT_PEAK, ANGLE_CURRENT_PEAK_DUR));
		rotation.configVoltageCompSaturation(VOLTAGE_COMP);
		rotation.configForwardSoftLimitEnable(false);

		rotation.config_kP(RobotMap.SLOT_INDEX, ANGLE_P);
		rotation.config_kI(RobotMap.SLOT_INDEX, ANGLE_I);
		rotation.config_kD(RobotMap.SLOT_INDEX, ANGLE_D);

		rotation.selectProfileSlot(RobotMap.SLOT_INDEX, RobotMap.LOOP_INDEX);

		rotation.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.SLOT_INDEX);
    }


    public void setPercentOutput(double output){
        rotation.set(ControlMode.PercentOutput, output);
    }

    public HSTalon getRotation(){
        return rotation;
    }

    public static Intake getInstance() {
        if (instance == null) {
           instance = new Intake();
        }
        return instance;
     }
}
