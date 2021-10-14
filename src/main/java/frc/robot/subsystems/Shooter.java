package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.util.Limelight;
import harkerrobolib.util.Conversions;
import harkerrobolib.util.Conversions.SpeedUnit;
import harkerrobolib.wrappers.HSFalcon;

public class Shooter extends SubsystemBase {
    private static Shooter instance;
    private HSFalcon rotation;
    private HSFalcon rotationFollower;

    private Servo hoodServo;

    private static final double ROTATION_P = 0.05;
    private static final double ROTATION_I = 0.0001;
    private static final double ROTATION_I_ZONE = 1700;

    
    private static final double ROTATION_D = 1;
    private static final double ROTATION_F = 0.064;
    private static final double RAMP_RATE = 0;
    private static final double VOLTAGE_COMP = 10;
    private static final double ANGLE_CURRENT_CONTINUOUS = 40;
    private static final double ANGLE_CURRENT_PEAK = 50;
    private static final double ANGLE_CURRENT_PEAK_DUR = 0.1;
    public static final int WHEEL_DIAMETER=4;

    public static final boolean ROTATION_INVERTED=true;

    public static final boolean ROTATION_FOLLOWER_INVERTED=false;
    public static final double GEAR_RATIO = 0.675;
    public static final double POWER_PORT_HEIGHT=8.1875;

    private static final int CURRENT_DRAW_MIN = 10;
    private static final int STALL_VELOCITY = 100;

    private Shooter() {
        rotation = new HSFalcon(RobotMap.SHOOTER_MASTER);
        rotationFollower = new HSFalcon(RobotMap.SHOOTER_FOLLOWER);

        hoodServo = new Servo(RobotMap.HOOD_SERVO_CHANNEL);
        
        intakeInit();
    }

    public void intakeInit(){
        rotation.configFactoryDefault();
        rotation.setNeutralMode(NeutralMode.Coast);
		rotation.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, ANGLE_CURRENT_CONTINUOUS, ANGLE_CURRENT_PEAK, ANGLE_CURRENT_PEAK_DUR));
		rotation.configVoltageCompSaturation(VOLTAGE_COMP);
		rotation.configForwardSoftLimitEnable(false);

		rotation.config_kP(RobotMap.SLOT_INDEX, ROTATION_P);
		rotation.config_kI(RobotMap.SLOT_INDEX, ROTATION_I);
        rotation.config_kD(RobotMap.SLOT_INDEX, ROTATION_D);
        rotation.config_kF(RobotMap.SLOT_INDEX, ROTATION_F);
        rotation.config_IntegralZone(RobotMap.SLOT_INDEX, ROTATION_I_ZONE);
        rotation.configClosedloopRamp(RAMP_RATE);

		rotation.selectProfileSlot(RobotMap.SLOT_INDEX, RobotMap.LOOP_INDEX);

        rotation.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, RobotMap.SLOT_INDEX, 100);
        rotation.setInverted(ROTATION_INVERTED);

        rotationFollower.configFactoryDefault();
        rotationFollower.follow(rotation);
        rotationFollower.setInverted(ROTATION_FOLLOWER_INVERTED);
    }


    public void setPercentOutput(double output){
        rotation.set(ControlMode.PercentOutput, output);
    }

    public void setVelocity(double output){
        rotation.set(ControlMode.Velocity, Conversions.convertSpeed(SpeedUnit.FEET_PER_SECOND, output, SpeedUnit.ENCODER_UNITS, WHEEL_DIAMETER, 2048) * GEAR_RATIO);
		
    }

    /**
     * @param servopos servo position from 0 to 0.8
     * @return angle in degrees (25-73)
     */
    public double servoToAngle(double servoPos){
        double x = servoPos;
        double angle = 72.0788 - 81.8362 * x + 22.8979 * x*x - 32.9049 * x*x*x + 7.46943 * x*x*x*x; 
        return angle;
    }

    /**
     * @param angle angle in degrees (25-73)
     * @return servo position from 0 to 0.8
     */
    public double angleToServo(double angle){
        double x = angle / 100;
        double servopos = 0.871253 - 0.961424 * x - 0.487348 * x*x - 0.0612509 * x*x*x + 0.3629 * x*x*x*x,
        return servopos;
    }

    /**
     * @return servo angle in degrees (25-73)
     */
    public double getHoodAngle() {
        return servoToAngle(hoodServo.get());
    }

    /**
     * @param angle hood angle in degrees (25-73)
     */
    public void setHoodAngle(double angle) {
        hoodServo.set(angleToServo(angle));
    }

    public boolean isStalling() {
        return (rotation.getStatorCurrent() > CURRENT_DRAW_MIN && rotation.getSelectedSensorVelocity() < STALL_VELOCITY) || 
                (rotationFollower.getStatorCurrent() > CURRENT_DRAW_MIN && rotationFollower.getSelectedSensorVelocity() < STALL_VELOCITY);
    }

    public HSFalcon getRotation(){
        return rotation;
    }
    
    public Servo getHoodServo() {
        return hoodServo;
    }
  

    public static Shooter getInstance() {
        if (instance == null) {
           instance = new Shooter();
        }
        return instance;
     }
}
