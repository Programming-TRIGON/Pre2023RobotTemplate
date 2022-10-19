package frc.trigon.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class SwerveConstants {
    public static final double MAX_SPEED_METERS_PER_SECOND = 4.25;
    public static final double MAX_ROTATIONAL_SPEED_RADIANS_PER_SECOND = 12.03;

    static final double DRIVE_RAMP_RATE = 0.6;
    static final double DEAD_BAND_DRIVE_DEADBAND_MPS = 0.1;
    private static final int PIGEON_ID = 25;
    public static final Pigeon2 gyro = new Pigeon2(PIGEON_ID);

    public static SwerveModule[] SWERVE_MODULES = {
            new SwerveModule(SwerveModuleConstants.SwerveModules.fromId(0).swerveModuleConstants),
            new SwerveModule(SwerveModuleConstants.SwerveModules.fromId(1).swerveModuleConstants),
            new SwerveModule(SwerveModuleConstants.SwerveModules.fromId(2).swerveModuleConstants),
            new SwerveModule(SwerveModuleConstants.SwerveModules.fromId(3).swerveModuleConstants)
    };

    private static final Translation2d[] LOCATIONS = {
            SwerveModuleConstants.SwerveModules.fromId(0).Location,
            SwerveModuleConstants.SwerveModules.fromId(1).Location,
            SwerveModuleConstants.SwerveModules.fromId(2).Location,
            SwerveModuleConstants.SwerveModules.fromId(3).Location
    };

    static SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(LOCATIONS);
}
