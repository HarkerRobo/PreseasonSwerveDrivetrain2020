package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import harkerrobolib.util.Conversions;
import harkerrobolib.util.Conversions.SpeedUnit;
import harkerrobolib.wrappers.HSFalcon;
import harkerrobolib.wrappers.HSTalon;

public class Shooter extends SubsystemBase {
    private static Shooter instance;
    private HSFalcon rotation;
    private HSFalcon rotation_follower;

    private DoubleSolenoid solenoid;
    private static final double ROTATION_P = 0.3;
    private static final double ROTATION_I = 0;
    
    private static final double ROTATION_D = 4;
    private static final double ROTATION_F = 0.05;
    private static final double RAMP_RATE = 0.1;
    private static final double VOLTAGE_COMP = 10;
    private static final double ANGLE_CURRENT_CONTINUOUS = 40;
    private static final double ANGLE_CURRENT_PEAK = 50;
    private static final double ANGLE_CURRENT_PEAK_DUR = 0.1;
    public static final int WHEEL_DIAMETER=4;
    public static final boolean INTAKE_INVERTED=true;

    public static final boolean ROTATION_INVERTED=true;

    public static final boolean ROTATION_FOLLOWER_INVERTED=false;


    private Shooter() {
        rotation=new HSFalcon(RobotMap.SHOOTER_MASTER);
        rotation_follower=new HSFalcon(RobotMap.SHOOTER_FOLLOWER);

        solenoid = new DoubleSolenoid(RobotMap.Shooter_SOLENOID_REVERSE, RobotMap.Shooter_SOLENOID_FORWARD);
        
        intakeInit();
    }

    public void intakeInit(){
        rotation.configFactoryDefault();
        rotation.setNeutralMode(NeutralMode.Brake);
		rotation.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, ANGLE_CURRENT_CONTINUOUS, ANGLE_CURRENT_PEAK, ANGLE_CURRENT_PEAK_DUR));
		rotation.configVoltageCompSaturation(VOLTAGE_COMP);
		rotation.configForwardSoftLimitEnable(false);

		rotation.config_kP(RobotMap.SLOT_INDEX, ROTATION_P);
		rotation.config_kI(RobotMap.SLOT_INDEX, ROTATION_I);
        rotation.config_kD(RobotMap.SLOT_INDEX, ROTATION_D);
        rotation.config_kF(RobotMap.SLOT_INDEX, ROTATION_F);
        rotation.configClosedloopRamp(RAMP_RATE);

		rotation.selectProfileSlot(RobotMap.SLOT_INDEX, RobotMap.LOOP_INDEX);

        rotation.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, RobotMap.SLOT_INDEX, 100);
        rotation.setInverted(ROTATION_INVERTED);

        rotation_follower.configFactoryDefault();
        rotation_follower.follow(rotation);
        rotation_follower.setInverted(ROTATION_FOLLOWER_INVERTED);
    }


    public void setPercentOutput(double output){
        rotation.set(ControlMode.PercentOutput, output);
    }

    public void setVelocity(double output){
        rotation.set(ControlMode.Velocity, Conversions.convertSpeed(SpeedUnit.FEET_PER_SECOND,  output, SpeedUnit.ENCODER_UNITS, WHEEL_DIAMETER, 4096));
		
    }

    public HSFalcon getRotation(){
        return rotation;
    }

    public DoubleSolenoid getSolenoid(){
        return solenoid;
    }

    public void invertSolenoid(){
        if(solenoid.get()==Value.kReverse){
            solenoid.set(Value.kForward);
            return;
        }
        solenoid.set(Value.kReverse);
    }
    public void setShooterForward(){
        solenoid.set(Value.kForward);
    }
    public void setShooterReverse(){
        solenoid.set(Value.kReverse);
    }

    public static Shooter getInstance() {
        if (instance == null) {
           instance = new Shooter();
        }
        return instance;
     }
}
