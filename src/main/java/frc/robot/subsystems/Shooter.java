package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.util.HSMotorFeedForward;
import frc.robot.util.Limelight;
import harkerrobolib.util.Conversions;
import harkerrobolib.util.Conversions.SpeedUnit;
import harkerrobolib.wrappers.HSFalcon;

public class Shooter extends SubsystemBase {
    private static Shooter instance;
    private HSFalcon rotation;
    private HSFalcon rotationFollower;

    private Servo hoodServo;

    private static final double ROTATION_P = 3;
    private static final double ROTATION_I = 0.0001;
    private static final double ROTATION_I_ZONE = 150;
    private static final double ROTATION_D = 0.9;
    private static final double ROTATION_F = 0;

    private static final double RAMP_RATE = 0;
    private static final double VOLTAGE_COMP = 10;

    private static final double ANGLE_CURRENT_CONTINUOUS = 50;
    private static final double ANGLE_CURRENT_PEAK = 60;
    private static final double ANGLE_CURRENT_PEAK_DUR = 0.2;
 
    public static final int WHEEL_DIAMETER= 6;

    public static final boolean ROTATION_INVERTED=true;
    public static boolean isPercentOutput = false;

    public static final boolean ROTATION_FOLLOWER_INVERTED=false;
    public static final double GEAR_RATIO = 2.0/3.0;
    public static final double POWER_PORT_HEIGHT= 8.1875;

    public static final double POWER_PORT_HEIGHT_test= 4.25;

    private static final int CURRENT_DRAW_MIN = 10;
    private static final int STALL_VELOCITY = 100;

    public static final double MAX_VEL = 220;

    private static final int NUM_SAMPLES = 30;
    public static MedianFilter medianFilter = new MedianFilter(NUM_SAMPLES);

    public static final double DAY_FAR_DISTANCE_THRESHOLD = 14.55;    
    public static final double DAY_MEDIUM_DISTANCE_THRESHOLD = 11.753;

    public double velAdjustment = -1;
    public double highVelAdjustment = 0;

    public HSMotorFeedForward feedForward;

    private Shooter() {
        rotation = new HSFalcon(RobotMap.SHOOTER_MASTER);
        rotationFollower = new HSFalcon(RobotMap.SHOOTER_FOLLOWER);

        hoodServo = new Servo(RobotMap.HOOD_SERVO_CHANNEL);
        feedForward = new HSMotorFeedForward(0.734, 0.0728, 0.0247);
        intakeInit();
    }

    public void periodic() {
        double limelightDistance = Shooter.getInstance().getDistance();
        double hoodAngle = 0.376581 + (0.00635765 * limelightDistance) + (-0.00001741 * Math.pow(limelightDistance, 2));
        if(limelightDistance==0) return;
       SmartDashboard.putNumber("limelight dist", limelightDistance);
       if(!OI.getInstance().getOperatorGamepad().getButtonYState()){
            Shooter.getInstance().setHoodAngle(hoodAngle);
        }
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
        double velocity = Conversions.convertSpeed(SpeedUnit.FEET_PER_SECOND, output, SpeedUnit.ENCODER_UNITS, WHEEL_DIAMETER, 2048) * GEAR_RATIO;
        if(rotation.getSelectedSensorVelocity() < 0.95 * velocity){
            rotation.set(ControlMode.PercentOutput, 1);
            isPercentOutput = true;
        }
        else {
            rotation.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedForward.calculate(velocity));
            isPercentOutput = false;
        }
    }

    public double getVelocity() {
        return Conversions.convertSpeed(SpeedUnit.ENCODER_UNITS, rotation.getSelectedSensorVelocity(), SpeedUnit.FEET_PER_SECOND, WHEEL_DIAMETER, 2048) / GEAR_RATIO;
    }

    /**
     * @return servo angle in degrees (25-73)
     */
    public double getHoodAngle() {
        return (hoodServo.get());
    }

    /**
     * @param angle hood angle in degrees (25-73)
     */
    public void setHoodAngle(double angle) {
        hoodServo.set(angle);
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

    public double getDistance() {
        if(Limelight.isTargetVisible()) {
            return medianFilter.calculate(POWER_PORT_HEIGHT - Limelight.LIMELIGHT_HEIGHT) / Math.tan(Math.toRadians(Limelight.getTy() + Limelight.LIMELIGHT_ANGLE));
        } else {
            medianFilter.reset();
            return 0;
        }
    }

    public double getYDistance() {

        
        return Math.sin(Math.toRadians(Drivetrain.getInstance().getPigeon().getFusedHeading() +  Limelight.getTx())) * getDistance();
    }

    public double getXDistance() {
       return Math.cos(Math.toRadians((Drivetrain.getInstance().getPigeon().getFusedHeading() +  Limelight.getTx() ))) * getDistance();
    }

    public double getVelocityError (){
        return rotation.getClosedLoopError();
    }
  

    public static Shooter getInstance() {
        if (instance == null) {
           instance = new Shooter();
        }
        return instance;
     }
}
