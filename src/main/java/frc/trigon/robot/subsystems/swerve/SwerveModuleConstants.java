package frc.trigon.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveModuleConstants {
    public static final double WHEEL_RADIUS_M = 0.05;
    public static final double DRIVE_GEAR_RATIO = 8.14;
    private static final double VOLTAGE_COMP_SATURATION = 12;

    static final double SIDE_LENGTH_M = 0.5;
    static final double DISTANCE_FROM_CENTER_OF_BASE = SIDE_LENGTH_M / 2;

    private static final int FRONT_LEFT_ID = 0;
    private static final int FRONT_RIGHT_ID = 1;
    private static final int REAR_LEFT_ID = 2;
    private static final int REAR_RIGHT_ID = 3;

    private static final boolean FRONT_LEFT_ANGLE_ENCODER_INVERTED = false;
    private static final boolean FRONT_LEFT_DRIVE_MOTOR_INVERTED = true;
    private static final boolean FRONT_LEFT_ANGLE_MOTOR_INVERTED = false;

    private static final boolean FRONT_RIGHT_ANGLE_ENCODER_INVERTED = false;
    private static final boolean FRONT_RIGHT_DRIVE_MOTOR_INVERTED = true;
    private static final boolean FRONT_RIGHT_ANGLE_MOTOR_INVERTED = false;

    private static final boolean REAR_LEFT_ANGLE_ENCODER_INVERTED = false;
    private static final boolean REAR_LEFT_DRIVE_MOTOR_INVERTED = true;
    private static final boolean REAR_LEFT_ANGLE_MOTOR_INVERTED = false;

    private static final boolean REAR_RIGHT_ANGLE_ENCODER_INVERTED = false;
    private static final boolean REAR_RIGHT_DRIVE_MOTOR_INVERTED = true;
    private static final boolean REAR_RIGHT_ANGLE_MOTOR_INVERTED = false;

    private static final int FRONT_LEFT_ANGLE_ENCODER_ID = 13;
    private static final int FRONT_LEFT_DRIVE_MOTOR_ID = 17;
    private static final int FRONT_LEFT_ANGLE_MOTOR_ID = 19;

    private static final int FRONT_RIGHT_ANGLE_ENCODER_ID = 5;
    private static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 1;
    private static final int FRONT_RIGHT_ANGLE_MOTOR_ID = 0;

    private static final int REAR_LEFT_ANGLE_ENCODER_ID = 16;
    private static final int REAR_LEFT_DRIVE_MOTOR_ID = 15;
    private static final int REAR_LEFT_ANGLE_MOTOR_ID = 14;

    private static final int REAR_RIGHT_ANGLE_ENCODER_ID = 3;
    private static final int REAR_RIGHT_DRIVE_MOTOR_ID = 2;
    private static final int REAR_RIGHT_ANGLE_MOTOR_ID = 4;

    private static final WPI_TalonSRX FRONT_LEFT_ANGLE_ENCODER = new WPI_TalonSRX(FRONT_LEFT_ANGLE_ENCODER_ID);
    private static final WPI_TalonFX FRONT_LEFT_DRIVE_MOTOR = new WPI_TalonFX(FRONT_LEFT_DRIVE_MOTOR_ID);
    private static final WPI_TalonFX FRONT_LEFT_ANGLE_MOTOR = new WPI_TalonFX(FRONT_LEFT_ANGLE_MOTOR_ID);
    private static final double FRONT_LEFT_ENCODER_OFFSET = 3501;

    private static final WPI_TalonSRX FRONT_RIGHT_ANGLE_ENCODER = new WPI_TalonSRX(FRONT_RIGHT_ANGLE_ENCODER_ID);
    private static final WPI_TalonFX FRONT_RIGHT_DRIVE_MOTOR = new WPI_TalonFX(FRONT_RIGHT_DRIVE_MOTOR_ID);
    private static final WPI_TalonFX FRONT_RIGHT_ANGLE_MOTOR = new WPI_TalonFX(FRONT_RIGHT_ANGLE_MOTOR_ID);
    private static final double FRONT_RIGHT_ENCODER_OFFSET = 2639;

    private static final WPI_TalonSRX REAR_LEFT_ANGLE_ENCODER = new WPI_TalonSRX(REAR_LEFT_ANGLE_ENCODER_ID);
    private static final WPI_TalonFX REAR_LEFT_DRIVE_MOTOR = new WPI_TalonFX(REAR_LEFT_DRIVE_MOTOR_ID);
    private static final WPI_TalonFX REAR_LEFT_ANGLE_MOTOR = new WPI_TalonFX(REAR_LEFT_ANGLE_MOTOR_ID);
    private static final double RIGHT_LEFT_ENCODER_OFFSET = 1673;

    private static final WPI_TalonSRX REAR_RIGHT_ANGLE_ENCODER = new WPI_TalonSRX(REAR_RIGHT_ANGLE_ENCODER_ID);
    private static final WPI_TalonFX REAR_RIGHT_DRIVE_MOTOR = new WPI_TalonFX(REAR_RIGHT_DRIVE_MOTOR_ID);
    private static final WPI_TalonFX REAR_RIGHT_ANGLE_MOTOR = new WPI_TalonFX(REAR_RIGHT_ANGLE_MOTOR_ID);
    private static final double REAR_RIGHT_ENCODER_OFFSET = 315;

    static final SwerveModuleConstants FRONT_LEFT_SWERVE_MODULE_CONSTANTS = new SwerveModuleConstants(
            FRONT_LEFT_ANGLE_ENCODER,
            FRONT_LEFT_DRIVE_MOTOR,
            FRONT_LEFT_ANGLE_MOTOR,
            FRONT_LEFT_ENCODER_OFFSET
    );
    static final SwerveModuleConstants FRONT_RIGHT_SWERVE_MODULE_CONSTANTS = new SwerveModuleConstants(
            FRONT_RIGHT_ANGLE_ENCODER,
            FRONT_RIGHT_DRIVE_MOTOR,
            FRONT_RIGHT_ANGLE_MOTOR,
            FRONT_RIGHT_ENCODER_OFFSET
    );
    static final SwerveModuleConstants REAR_LEFT_SWERVE_MODULE_CONSTANTS = new SwerveModuleConstants(
            REAR_LEFT_ANGLE_ENCODER,
            REAR_LEFT_DRIVE_MOTOR,
            REAR_LEFT_ANGLE_MOTOR,
            RIGHT_LEFT_ENCODER_OFFSET
    );
    static final SwerveModuleConstants REAR_RIGHT_SWERVE_MODULE_CONSTANTS = new SwerveModuleConstants(
            REAR_RIGHT_ANGLE_ENCODER,
            REAR_RIGHT_DRIVE_MOTOR,
            REAR_RIGHT_ANGLE_MOTOR,
            REAR_RIGHT_ENCODER_OFFSET
    );

    static {
        FRONT_LEFT_ANGLE_ENCODER.configFactoryDefault();
        FRONT_LEFT_DRIVE_MOTOR.configFactoryDefault();
        FRONT_LEFT_ANGLE_MOTOR.configFactoryDefault();

        FRONT_RIGHT_ANGLE_ENCODER.configFactoryDefault();
        FRONT_RIGHT_DRIVE_MOTOR.configFactoryDefault();
        FRONT_RIGHT_ANGLE_MOTOR.configFactoryDefault();

        REAR_LEFT_ANGLE_ENCODER.configFactoryDefault();
        REAR_LEFT_DRIVE_MOTOR.configFactoryDefault();
        REAR_LEFT_ANGLE_MOTOR.configFactoryDefault();

        REAR_RIGHT_ANGLE_ENCODER.configFactoryDefault();
        REAR_RIGHT_DRIVE_MOTOR.configFactoryDefault();
        REAR_RIGHT_ANGLE_MOTOR.configFactoryDefault();

        FRONT_LEFT_ANGLE_ENCODER.setInverted(FRONT_LEFT_ANGLE_ENCODER_INVERTED);
        FRONT_LEFT_DRIVE_MOTOR.setInverted(FRONT_LEFT_DRIVE_MOTOR_INVERTED);
        FRONT_LEFT_ANGLE_MOTOR.setInverted(FRONT_LEFT_ANGLE_MOTOR_INVERTED);

        FRONT_RIGHT_ANGLE_ENCODER.setInverted(FRONT_RIGHT_ANGLE_ENCODER_INVERTED);
        FRONT_RIGHT_DRIVE_MOTOR.setInverted(FRONT_RIGHT_DRIVE_MOTOR_INVERTED);
        FRONT_RIGHT_ANGLE_MOTOR.setInverted(FRONT_RIGHT_ANGLE_MOTOR_INVERTED);

        REAR_LEFT_ANGLE_ENCODER.setInverted(REAR_LEFT_ANGLE_ENCODER_INVERTED);
        REAR_LEFT_DRIVE_MOTOR.setInverted(REAR_LEFT_DRIVE_MOTOR_INVERTED);
        REAR_LEFT_ANGLE_MOTOR.setInverted(REAR_LEFT_ANGLE_MOTOR_INVERTED);

        REAR_RIGHT_ANGLE_ENCODER.setInverted(REAR_RIGHT_ANGLE_ENCODER_INVERTED);
        REAR_RIGHT_DRIVE_MOTOR.setInverted(REAR_RIGHT_DRIVE_MOTOR_INVERTED);
        REAR_RIGHT_ANGLE_MOTOR.setInverted(REAR_RIGHT_ANGLE_MOTOR_INVERTED);

        FRONT_LEFT_DRIVE_MOTOR.configOpenloopRamp(SwerveConstants.DRIVE_RAMP_RATE);
        FRONT_RIGHT_DRIVE_MOTOR.configOpenloopRamp(SwerveConstants.DRIVE_RAMP_RATE);
        FRONT_RIGHT_DRIVE_MOTOR.configOpenloopRamp(SwerveConstants.DRIVE_RAMP_RATE);
        REAR_RIGHT_DRIVE_MOTOR.configOpenloopRamp(SwerveConstants.DRIVE_RAMP_RATE);

        FRONT_LEFT_DRIVE_MOTOR.configVoltageCompSaturation(VOLTAGE_COMP_SATURATION);
        FRONT_LEFT_ANGLE_MOTOR.configVoltageCompSaturation(VOLTAGE_COMP_SATURATION);

        FRONT_RIGHT_DRIVE_MOTOR.configVoltageCompSaturation(VOLTAGE_COMP_SATURATION);
        FRONT_RIGHT_ANGLE_MOTOR.configVoltageCompSaturation(VOLTAGE_COMP_SATURATION);

        REAR_LEFT_DRIVE_MOTOR.configVoltageCompSaturation(VOLTAGE_COMP_SATURATION);
        REAR_LEFT_ANGLE_MOTOR.configVoltageCompSaturation(VOLTAGE_COMP_SATURATION);

        REAR_RIGHT_DRIVE_MOTOR.configVoltageCompSaturation(VOLTAGE_COMP_SATURATION);
        REAR_RIGHT_ANGLE_MOTOR.configVoltageCompSaturation(VOLTAGE_COMP_SATURATION);

        FRONT_LEFT_DRIVE_MOTOR.enableVoltageCompensation(true);
        FRONT_LEFT_ANGLE_MOTOR.enableVoltageCompensation(true);

        FRONT_RIGHT_DRIVE_MOTOR.enableVoltageCompensation(true);
        FRONT_RIGHT_ANGLE_MOTOR.enableVoltageCompensation(true);

        REAR_LEFT_DRIVE_MOTOR.enableVoltageCompensation(true);
        REAR_LEFT_ANGLE_MOTOR.enableVoltageCompensation(true);

        REAR_RIGHT_DRIVE_MOTOR.enableVoltageCompensation(true);
        REAR_RIGHT_ANGLE_MOTOR.enableVoltageCompensation(true);

        FRONT_RIGHT_DRIVE_MOTOR.setStatusFramePeriod(StatusFrame.Status_1_General, 1000, 0);
        FRONT_LEFT_DRIVE_MOTOR.setStatusFramePeriod(StatusFrame.Status_1_General, 1000, 0);
        REAR_RIGHT_DRIVE_MOTOR.setStatusFramePeriod(StatusFrame.Status_1_General, 1000, 0);
        REAR_LEFT_DRIVE_MOTOR.setStatusFramePeriod(StatusFrame.Status_1_General, 1000, 0);

        FRONT_RIGHT_ANGLE_MOTOR.setStatusFramePeriod(StatusFrame.Status_1_General, 1000, 0);
        FRONT_RIGHT_ANGLE_MOTOR.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 1000, 0);
        FRONT_LEFT_ANGLE_MOTOR.setStatusFramePeriod(StatusFrame.Status_1_General, 1000, 0);
        FRONT_LEFT_ANGLE_MOTOR.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 1000, 0);
        REAR_RIGHT_ANGLE_MOTOR.setStatusFramePeriod(StatusFrame.Status_1_General, 1000, 0);
        REAR_RIGHT_ANGLE_MOTOR.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 1000, 0);
        REAR_LEFT_ANGLE_MOTOR.setStatusFramePeriod(StatusFrame.Status_1_General, 1000, 0);
        REAR_LEFT_ANGLE_MOTOR.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 1000, 0);
    }

    public static double
            FRONT_LEFT_P = 0.7,
            FRONT_LEFT_I = 0,
            FRONT_LEFT_D = 0;

    public static double
            FRONT_RIGHT_P = 0.7,
            FRONT_RIGHT_I = 0,
            FRONT_RIGHT_D = 0;

    public static double
            REAR_LEFT_P = 0.7,
            REAR_LEFT_I = 0,
            REAR_LEFT_D = 0;

    public static double
            REAR_RIGHT_P = 0.7,
            REAR_RIGHT_I = 0,
            REAR_RIGHT_D = 0;

    static {
        FRONT_LEFT_ANGLE_MOTOR.config_kP(0, FRONT_LEFT_P);
        FRONT_LEFT_ANGLE_MOTOR.config_kI(0, FRONT_LEFT_I);
        FRONT_LEFT_ANGLE_MOTOR.config_kD(0, FRONT_LEFT_D);

        FRONT_RIGHT_ANGLE_MOTOR.config_kP(0, FRONT_RIGHT_P);
        FRONT_RIGHT_ANGLE_MOTOR.config_kI(0, FRONT_RIGHT_I);
        FRONT_RIGHT_ANGLE_MOTOR.config_kD(0, FRONT_RIGHT_D);

        REAR_LEFT_ANGLE_MOTOR.config_kP(0, REAR_LEFT_P);
        REAR_LEFT_ANGLE_MOTOR.config_kI(0, REAR_LEFT_I);
        REAR_LEFT_ANGLE_MOTOR.config_kD(0, REAR_LEFT_D);

        REAR_RIGHT_ANGLE_MOTOR.config_kP(0, REAR_RIGHT_P);
        REAR_RIGHT_ANGLE_MOTOR.config_kI(0, REAR_RIGHT_I);
        REAR_RIGHT_ANGLE_MOTOR.config_kD(0, REAR_RIGHT_D);
    }

    private static final Translation2d FRONT_LEFT_MODULE_LOCATION = new Translation2d(
            DISTANCE_FROM_CENTER_OF_BASE,
            DISTANCE_FROM_CENTER_OF_BASE
    );
    private static final Translation2d FRONT_RIGHT_MODULE_LOCATION = new Translation2d(
            DISTANCE_FROM_CENTER_OF_BASE,
            -DISTANCE_FROM_CENTER_OF_BASE
    );
    private static final Translation2d REAR_LEFT_MODULE_LOCATION = new Translation2d(
            -DISTANCE_FROM_CENTER_OF_BASE,
            DISTANCE_FROM_CENTER_OF_BASE
    );
    private static final Translation2d REAR_RIGHT_MODULE_LOCATION = new Translation2d(
            -DISTANCE_FROM_CENTER_OF_BASE,
            -DISTANCE_FROM_CENTER_OF_BASE
    );

    public WPI_TalonSRX angleEncoder;
    public WPI_TalonFX angleMotor;
    public WPI_TalonFX driveMotor;
    public double encoderOffset;

    public SwerveModuleConstants(
            WPI_TalonSRX angleEncoder, WPI_TalonFX driveMotor, WPI_TalonFX angleMotor, double encoderOffset) {
        this.angleEncoder = angleEncoder;
        this.driveMotor = driveMotor;
        this.angleMotor = angleMotor;
        this.encoderOffset = encoderOffset;
    }

    public enum SwerveModules {
        FRONT_LEFT(FRONT_LEFT_ID, FRONT_LEFT_SWERVE_MODULE_CONSTANTS, FRONT_LEFT_MODULE_LOCATION),
        FRONT_RIGHT(FRONT_RIGHT_ID, FRONT_RIGHT_SWERVE_MODULE_CONSTANTS, FRONT_RIGHT_MODULE_LOCATION),
        REAR_LEFT(REAR_LEFT_ID, REAR_LEFT_SWERVE_MODULE_CONSTANTS, REAR_LEFT_MODULE_LOCATION),
        REAR_RIGHT(REAR_RIGHT_ID, REAR_RIGHT_SWERVE_MODULE_CONSTANTS, REAR_RIGHT_MODULE_LOCATION);

        final int id;
        final SwerveModuleConstants swerveModuleConstants;
        final Translation2d Location;

        SwerveModules(int id, SwerveModuleConstants swerveModuleConstants, Translation2d location) {
            this.id = id;
            this.swerveModuleConstants = swerveModuleConstants;
            this.Location = location;
        }

        public static SwerveModules fromId(int id) {
            return values()[id];
        }
    }
}

