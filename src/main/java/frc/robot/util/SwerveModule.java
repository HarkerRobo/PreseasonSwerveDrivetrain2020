package frc.robot.util;

import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;
import harkerrobolib.wrappers.HSTalon;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import harkerrobolib.util.Conversions.SpeedUnit;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import harkerrobolib.util.Conversions;
import harkerrobolib.wrappers.HSFalcon;

public class SwerveModule {
	public HSTalon rotation;
	public HSFalcon translation;

	private static final double VOLTAGE_COMP = 10;

	private static final double DRIVE_CURRENT_CONTINUOUS = 40;
    private static final double DRIVE_CURRENT_PEAK = 60;
    private static final double DRIVE_CURRENT_PEAK_DUR = 0.4;

    private static final double ANGLE_CURRENT_CONTINUOUS = 15;
    private static final double ANGLE_CURRENT_PEAK = 20;
    private static final double ANGLE_CURRENT_PEAK_DUR = 0.02;

	private static final double TRANSLATION_P = 0.7;
	private static final double TRANSLATION_I = 0.0;
	private static final double TRANSLATION_D = 10;
	private static final double TRANSLATION_F = 0; //0.04697602371;

	private static final double ANGLE_P = 1.1;
	private static final double ANGLE_I = 0;
	private static final double ANGLE_D = 11;
	private static final double ENCODER_TICKS = 4096.0;
	private static final double EPSILON_OUTPUT = 1e-4;
	private static final int DRIVE_TICKS_PER_REV = 2048;

	private boolean ROTATION_SENSOR_PHASE;
	private boolean TRANSLATION_SENSOR_PHASE;

	
	private boolean ROTATION_INVERT;
	private boolean TRANSLATION_INVERT;

	private HSMotorFeedForward feedForward;


	public SwerveModule(boolean rotationSensorPhase, boolean translationSensorPhase, int rotationDriveId,
			int translationDriveId, boolean rotationInverted, boolean translationInverted) {
		ROTATION_SENSOR_PHASE = rotationSensorPhase;
		TRANSLATION_SENSOR_PHASE = translationSensorPhase;

		ROTATION_INVERT=rotationInverted;
		TRANSLATION_INVERT=translationInverted;

		rotation = new HSTalon(rotationDriveId);
		translation = new HSFalcon(translationDriveId);
		feedForward = new HSMotorFeedForward(0.705, 0.0506, 0.00202);
		rotationMotorInit();
		translationMotorInit();
	}

	public HSFalcon getTranslationMotor() {
		return translation;
	}

	public HSTalon getRotationMotor() {
		return rotation;
	}

	public double getRotationAngle(){
		return rotation.getSelectedSensorPosition()*(360.0/4096);
	}

	private void rotationMotorInit() {
		rotation.configFactoryDefault();
		rotation.setInverted(ROTATION_INVERT);
		rotation.setSensorPhase(ROTATION_SENSOR_PHASE);
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

	private void translationMotorInit() {
		translation.configFactoryDefault();
		translation.setSensorPhase(TRANSLATION_SENSOR_PHASE);
		translation.setNeutralMode(NeutralMode.Brake);
		translation.setInverted(TRANSLATION_INVERT);

		translation.configClosedloopRamp(0.1);
		translation.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, DRIVE_CURRENT_CONTINUOUS, DRIVE_CURRENT_PEAK, DRIVE_CURRENT_PEAK_DUR));

		translation.config_kP(RobotMap.SLOT_INDEX, TRANSLATION_P);
		translation.config_kI(RobotMap.SLOT_INDEX, TRANSLATION_I);
		translation.config_kD(RobotMap.SLOT_INDEX, TRANSLATION_D);
		translation.config_kF(RobotMap.SLOT_INDEX, TRANSLATION_F);

		translation.selectProfileSlot(RobotMap.SLOT_INDEX, RobotMap.LOOP_INDEX);

		translation.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, RobotMap.SLOT_INDEX);

	}

	public void setPercentOutput(Vector translationvec) {
		translation.set(ControlMode.PercentOutput, translationvec.getMagnitude());
		rotation.set(ControlMode.Position, translationvec.getAngle() * (4096 / 360));

	}

	public SwerveModuleState getState() {
		return new SwerveModuleState(
			Conversions.convertSpeed(SpeedUnit.ENCODER_UNITS, translation.getSelectedSensorVelocity() / Drivetrain.FEET_TO_METER, SpeedUnit.FEET_PER_SECOND, Drivetrain.WHEEL_DIAMETER, 2048) / Drivetrain.GEAR_RATIO, 
			Rotation2d.fromDegrees(rotation.getSelectedSensorPosition() * 360 / 4096)
		);
	}

	public void setSwerveManual(SwerveModuleState state){
		//state = optimize(state, getRotationAngle());
		double speed=state.speedMetersPerSecond;
		double angle = state.angle.getDegrees();
		double currentAngle=getRotationAngle();
		while(angle-currentAngle>180){
			angle -= 360;
		}

		while(angle-currentAngle<-180){                                                                                             

			angle += 360;
		}

		if(angle-currentAngle>90){
			angle -= 180;
			speed *= -1;
		}

		else if(angle-currentAngle<-90){
			angle += 180;
			speed *= -1;
		}

		double vel = Drivetrain.GEAR_RATIO * Conversions.convertSpeed(SpeedUnit.FEET_PER_SECOND,  speed * Drivetrain.FEET_TO_METER, SpeedUnit.ENCODER_UNITS, Drivetrain.WHEEL_DIAMETER, 2048);

		translation.set(TalonFXControlMode.Velocity, vel, DemandType.ArbitraryFeedForward, feedForward.calculate(vel)/VOLTAGE_COMP);
		
		rotation.set(ControlMode.Position,angle * (4096 / 360));
	}
}