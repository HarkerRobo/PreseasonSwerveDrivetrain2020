package frc.robot.subsystems;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.util.SwerveModule;

import frc.robot.util.Vector;

public class Drivetrain extends SubsystemBase {
    private static Drivetrain drivetrain;

    private SwerveModule topLeft;
    private SwerveModule topRight;
    private SwerveModule bottomLeft;
    private SwerveModule bottomRight;

    private boolean TOP_LEFT_ROTATION_SENSOR_PHASE = false;
    private boolean TOP_LEFT_TRANSLATION_SENSOR_PHASE = false;
    private boolean TOP_LEFT_INVERTED = false;

    private boolean TOP_RIGHT_ROTATION_SENSOR_PHASE = false;
    private boolean TOP_RIGHT_TRANSLATION_SENSOR_PHASE = false;
    private boolean TOP_RIGHT_INVERTED = false;

    private boolean BOTTOM_LEFT_ROTATION_SENSOR_PHASE = false;
    private boolean BOTTOM_LEFT_TRANSLATION_SENSOR_PHASE = false;
    private boolean BOTTOM_LEFT_INVERTED = false;

    private boolean BOTTOM_RIGHT_ROTATION_SENSOR_PHASE = false;
    private boolean BOTTOM_RIGHT_TRANSLATION_SENSOR_PHASE = false;
    private boolean BOTTOM_RIGHT_INVERTED = false;

    private Translation2d m_frontLeftLocation;
    private Translation2d m_frontRightLocation;
    private Translation2d m_backLeftLocation;
    private Translation2d m_backRightLocation;
    private SwerveDriveKinematics m_kinematics;

    public static final double DT_WIDTH = 0.419;
    public static final double DT_LENGTH = 0.523;
    public static final double TALON_PEAK_LIMIT = 20;
    public static final double TALON_PEAK_TIME = 0.750;
    public static final double TALON_CONTINUOUS_LIMIT = 15;
    public static final double VOLTAGE_COMP = 10;

    public static final double MAX_SPEED = 0.7;

    public SwerveDriveKinematics swerveDriveKinematics= new SwerveDriveKinematics();

    public Drivetrain() {
        topLeft = new SwerveModule(TOP_LEFT_ROTATION_SENSOR_PHASE, TOP_LEFT_TRANSLATION_SENSOR_PHASE,
                RobotMap.ROTATION_IDS[0], RobotMap.TRANSLATION_IDS[0], TOP_LEFT_INVERTED);
        topRight = new SwerveModule(TOP_RIGHT_ROTATION_SENSOR_PHASE, TOP_RIGHT_TRANSLATION_SENSOR_PHASE,
                RobotMap.ROTATION_IDS[1], RobotMap.TRANSLATION_IDS[1], TOP_RIGHT_INVERTED);
        bottomLeft = new SwerveModule(BOTTOM_LEFT_ROTATION_SENSOR_PHASE, BOTTOM_LEFT_TRANSLATION_SENSOR_PHASE,
                RobotMap.ROTATION_IDS[2], RobotMap.TRANSLATION_IDS[2], BOTTOM_LEFT_INVERTED);
        bottomRight = new SwerveModule(BOTTOM_RIGHT_ROTATION_SENSOR_PHASE, BOTTOM_RIGHT_TRANSLATION_SENSOR_PHASE,
                RobotMap.ROTATION_IDS[3], RobotMap.TRANSLATION_IDS[3], BOTTOM_RIGHT_INVERTED);

        m_frontLeftLocation = new Translation2d(-DT_LENGTH / 2, DT_WIDTH / 2);
        m_frontRightLocation = new Translation2d(DT_LENGTH / 2, DT_WIDTH / 2);
        m_backLeftLocation = new Translation2d(-DT_LENGTH / 2, -DT_WIDTH / 2);
        m_backRightLocation = new Translation2d(DT_LENGTH / 2, -DT_WIDTH / 2);
        m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

        // In SwerveManual:

        // TODO: Add multiplier to raw outputs for safety

        // TODO: Make ChassisSpeeds object (vx, vy, omega)

        // TODO: Convert from ChassisSpeeds to SwerveModuleStates

        // In DriveTrain:
        
        // Set velocities based on SwerveModuleStates' speeds and angles,
        // make sure to convert from angle degrees to encoder units before
        // implementing angle PID
    }

    public SwerveModule getTopLeft() {
        return topLeft;
    }

    public SwerveModule getTopRight() {
        return topRight;
    }

    public SwerveModule getBottomLeft() {
        return bottomLeft;
    }

    public SwerveModule getBottomRight() {
        return bottomRight;
    }

    public void setPercentOutput(Vector translation){
        topLeft.setPercentOutput(translation);
        topRight.setPercentOutput(translation);
        bottomLeft.setPercentOutput(translation);
        bottomRight.setPercentOutput(translation);
    }

    public void setAngleAndDriveVelocity(SwerveModuleState[] states){
        topLeft.setSwerveManual(states[0]);
        topRight.setSwerveManual(states[1]);
        bottomLeft.setSwerveManual(states[2]);
        bottomRight.setSwerveManual(states[3]);
    }

    public SwerveDriveKinematics getSwerveDriveKinematics(){
        return swerveDriveKinematics;
    }

    public static Drivetrain getInstance() {
        if (drivetrain == null) {
            drivetrain = new Drivetrain();
        }
        return drivetrain;
    }
}
