package frc.robot.util;

import frc.robot.RobotMap;
import harkerrobolib.wrappers.HSTalon;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

import harkerrobolib.wrappers.HSFalcon;

public class SwerveModule {
    public HSTalon rotation;
    public HSFalcon translation;

    private static final int CURRENT_LIMIT = 0;
    private static final double VOLTAGE_COMP = 0;

    private static final int FALCON_CURRENT_LIMIT = 0;
    private static final double CURRENT_TRIGGER_THRESHOLD = 0;
    private static final double TRIGGER_THRESHOLD_TIME = 0;

    private static final double TRANSLATION_P = 0;
    private static final double TRANSLATION_I = 0;
    private static final double TRANSLATION_D = 0;

    private boolean ROTATION_SENSOR_PHASE;
    private boolean TRANSLATION_SENSOR_PHASE;

    public SwerveModule(boolean rotationSensorPhase, boolean translationSensorPhase, int[] driveIds) {
        ROTATION_SENSOR_PHASE = rotationSensorPhase;
        TRANSLATION_SENSOR_PHASE = translationSensorPhase;

        rotation = new HSTalon(driveIds[0]);
        translation = new HSFalcon(driveIds[1]);
        rotationMotorInit();
        translationMotorInit();
    }

    public HSFalcon getTranslationMotor() {
        return translation;
    }

    public HSTalon getRotationMotor() {
        return rotation;
    }

    private void rotationMotorInit() {
        rotation.configFactoryDefault();
        rotation.setSensorPhase(ROTATION_SENSOR_PHASE);
        rotation.setNeutralMode(NeutralMode.Brake);
        rotation.configContinuousCurrentLimit(CURRENT_LIMIT, 0);
        rotation.configVoltageCompSaturation(VOLTAGE_COMP);
        rotation.configForwardSoftLimitEnable(true);


        rotation.config_kP(RobotMap.SLOT_INDEX, TRANSLATION_P);
        rotation.config_kI(RobotMap.SLOT_INDEX, TRANSLATION_I);
        rotation.config_kD(RobotMap.SLOT_INDEX, TRANSLATION_D);

        rotation.selectProfileSlot(RobotMap.SLOT_INDEX, RobotMap.LOOP_INDEX);

        rotation.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.LOOP_INDEX);


    }

    private void translationMotorInit() {
        translation.configFactoryDefault();
        translation.setSensorPhase(TRANSLATION_SENSOR_PHASE);
        translation.setNeutralMode(NeutralMode.Coast);

        translation.config_kP(RobotMap.SLOT_INDEX, TRANSLATION_P);
        translation.config_kI(RobotMap.SLOT_INDEX, TRANSLATION_I);
        translation.config_kD(RobotMap.SLOT_INDEX, TRANSLATION_D);

        translation.selectProfileSlot(RobotMap.SLOT_INDEX, RobotMap.LOOP_INDEX);

        translation.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.LOOP_INDEX);



        translation.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, FALCON_CURRENT_LIMIT,
                CURRENT_TRIGGER_THRESHOLD, TRIGGER_THRESHOLD_TIME));
    }

    public void setPercentOutput(double speed, double turn) {
        translation.set(ControlMode.PercentOutput, speed);
        setRotationMotor(getAngle(speed, turn));
    }

    public void setRotationMotor(double angle) {
        rotation.set(ControlMode.Position, angle);
    }

    private double getAngle(double speed, double turn) {
        return Math.atan2(speed, turn);
    }
}
