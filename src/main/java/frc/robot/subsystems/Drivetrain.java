package frc.robot.subsystems;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.util.SwerveModule;

import frc.robot.util.Vector;

import harkerrobolib.wrappers.HSPigeon;

public class Drivetrain extends SubsystemBase {
    private static Drivetrain drivetrain;

    private SwerveModule topLeft;
    private SwerveModule topRight;
    private SwerveModule bottomLeft;
    private SwerveModule bottomRight;

    private static final boolean TOP_LEFT_ROTATION_SENSOR_PHASE = true;//false;
    private static final boolean TOP_LEFT_TRANSLATION_SENSOR_PHASE = false;
    private static final boolean TOP_LEFT_ROTATION_INVERTED = true;
    private static final boolean TOP_LEFT_TRANSLATION_INVERTED = false;

    private static final boolean TOP_RIGHT_ROTATION_SENSOR_PHASE = true;
    private static final boolean TOP_RIGHT_TRANSLATION_SENSOR_PHASE = false;
    private static final boolean TOP_RIGHT_ROTATION_INVERTED = true;
    private static final boolean TOP_RIGHT_TRANSLATION_INVERTED = false;

    private static final boolean BOTTOM_LEFT_ROTATION_SENSOR_PHASE = true;
    private static final boolean BOTTOM_LEFT_TRANSLATION_SENSOR_PHASE = false;
    private static final boolean BOTTOM_LEFT_ROTATION_INVERTED = true; //true;
    private static final boolean BOTTOM_LEFT_TRANSLATION_INVERTED = false;

    private static final boolean BOTTOM_RIGHT_ROTATION_SENSOR_PHASE = false;
    private static final boolean BOTTOM_RIGHT_TRANSLATION_SENSOR_PHASE = false;
    private static final boolean BOTTOM_RIGHT_ROTATION_INVERTED = false; //false;
    private static final boolean BOTTOM_RIGHT_TRANSLATION_INVERTED = false;

    public static final int TL_OFFSET = 2327-2048+40;
    public static final int TR_OFFSET = 1538+2048;
    public static final int BL_OFFSET = 408+2048-20;
    public static final int BR_OFFSET = 1465+2048;

    private Translation2d frontLeftLocation;
    private Translation2d frontRightLocation;
    private Translation2d backLeftLocation;
    private Translation2d backRightLocation;

    public static final double DT_WIDTH = 0.419;
    public static final double DT_LENGTH = 0.523;
    public static final double TALON_PEAK_LIMIT = 20;
    public static final double TALON_PEAK_TIME = 0.750;
    public static final double TALON_CONTINUOUS_LIMIT = 15;
    public static final double VOLTAGE_COMP = 10;

    public static final double MAX_DRIVE_VEL = 2;
    public static final double MAX_ANGULAR_VEL = 1 * Math.PI;

    public static final double FEET_TO_METER=3.281;
    public static final int WHEEL_DIAMETER=4;
    public static final int GEAR_RATIO=6;
    public static final double MIN_OUTPUT=10e-4;

    public static final double AUTO_MAX_SPEED = 2;
    public static final double AUTO_MAX_ANGULAR_VEL = Math.PI;

    public static final double AUTO_MAX_SPEED_ACCELERATION = 1;
    public static final double AUTO_MAX_ANGULAR_VEL_ACCELERATION = 0.5 * Math.PI;

    private HSPigeon pigeon;

    private SwerveDriveKinematics kinematics;

    private SwerveDriveOdometry odometry;

    public Drivetrain() {
        topLeft = new SwerveModule(TOP_LEFT_ROTATION_SENSOR_PHASE, TOP_LEFT_TRANSLATION_SENSOR_PHASE,
                RobotMap.ROTATION_IDS[0], RobotMap.TRANSLATION_IDS[0], TOP_LEFT_ROTATION_INVERTED, TOP_LEFT_TRANSLATION_INVERTED);
        topRight = new SwerveModule(TOP_RIGHT_ROTATION_SENSOR_PHASE, TOP_RIGHT_TRANSLATION_SENSOR_PHASE,
                RobotMap.ROTATION_IDS[1], RobotMap.TRANSLATION_IDS[1], TOP_RIGHT_ROTATION_INVERTED, TOP_RIGHT_TRANSLATION_INVERTED);
        bottomLeft = new SwerveModule(BOTTOM_LEFT_ROTATION_SENSOR_PHASE, BOTTOM_LEFT_TRANSLATION_SENSOR_PHASE,
                RobotMap.ROTATION_IDS[2], RobotMap.TRANSLATION_IDS[2], BOTTOM_LEFT_ROTATION_INVERTED, BOTTOM_LEFT_TRANSLATION_INVERTED);
        bottomRight = new SwerveModule(BOTTOM_RIGHT_ROTATION_SENSOR_PHASE, BOTTOM_RIGHT_TRANSLATION_SENSOR_PHASE,
                RobotMap.ROTATION_IDS[3], RobotMap.TRANSLATION_IDS[3], BOTTOM_RIGHT_ROTATION_INVERTED, BOTTOM_RIGHT_TRANSLATION_INVERTED);
        
        pigeon = new HSPigeon(RobotMap.PIGEON_ID);
        pigeon.setFusedHeading(0);

        frontLeftLocation = new Translation2d(-DT_LENGTH / 2, DT_WIDTH / 2);
        frontRightLocation = new Translation2d(DT_LENGTH / 2, DT_WIDTH / 2);
        backLeftLocation = new Translation2d(-DT_LENGTH / 2, -DT_WIDTH / 2);
        backRightLocation = new Translation2d(DT_LENGTH / 2, -DT_WIDTH / 2);
        
        kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);
        odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(pigeon.getFusedHeading()), new Pose2d(0,0, new Rotation2d()));
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

    public HSPigeon getPigeon(){
        return pigeon;
    }

    public void setPercentOutput(Vector translation){
        topLeft.setPercentOutput(translation);
        topRight.setPercentOutput(translation);
        bottomLeft.setPercentOutput(translation);
        bottomRight.setPercentOutput(translation);
    }

    public void setAngleAndDriveVelocity(SwerveModuleState[] states){
        topLeft.setSwerveManual(states[0]);
        SmartDashboard.putNumber("TL VEL", states[0].speedMetersPerSecond);
        topRight.setSwerveManual(states[1]);
        bottomLeft.setSwerveManual(states[2]);
        bottomRight.setSwerveManual(states[3]);
    }

    public SwerveDriveOdometry getOdometry() {
        return odometry;
    }

    public SwerveDriveKinematics getKinematics(){
        return kinematics;
    }

    public static Drivetrain getInstance() {
        if (drivetrain == null) {
            drivetrain = new Drivetrain();
        }
        return drivetrain;
    }
}
