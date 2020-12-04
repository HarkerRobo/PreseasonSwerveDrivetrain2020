import harkerrobolib.wrappers.HSTalon;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import harkerrobolib.wrappers.HSFalcon;

public class SwerveModule {
    public HSTalon rotation;
    public HSFalcon translation;

    private static final boolean ROTATION_SENSOR_PHASE = false;
    private static final boolean TRANSLATION_SENSOR_PHASE = false;

    public SwerveModule() {
        rotation = new HSTalon(RobotMap.DRIVE_IDS[0]);
        translation = new HSFalcon(RobotMap.DRIVE_IDS[0]);
        rotationMotorInit();
        translationMotorInit();
    }

    private void rotationMotorInit() {
        rotation.configFactoryDefault();
        rotation.setSensorPhase(ROTATION_SENSOR_PHASE);
        rotation.setNeutralMode(NeutralMode.Brake);
    }

    private void translationMotorInit() {
        translation.configFactoryDefault();
        translation.setSensorPhase(TRANSLATION_SENSOR_PHASE);
        translation.setNeutralMode(NeutralMode.Coast);

    }
}
