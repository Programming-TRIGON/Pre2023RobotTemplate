package frc.trigon.robot.subsystems.swerve;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    private static final Swerve INSTANCE = new Swerve();
    private final SwerveDrivePoseEstimator poseEstimator =
            new SwerveDrivePoseEstimator(
                    new Rotation2d(),
                    new Pose2d(),
                    SwerveConstants.KINEMATICS,
                    VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5)),
                    VecBuilder.fill(Units.degreesToRadians(0.1)),
                    SwerveConstants.VISION_STD_DEVS
            );


    private Swerve() {
        zeroHeading();
        putOnDashboard();
    }

    public static Swerve getInstance() {
        return INSTANCE;
    }

    /**
     * Drives the swerve with the given velocities, relative to the robot.
     *
     * @param translation the target x and y velocities in m/s
     * @param rotation    the target theta velocity in radians per second
     */
    void selfRelativeDrive(Translation2d translation, Rotation2d rotation) {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
                translation.getX(),
                translation.getY(),
                rotation.getRadians()
        );
        selfRelativeDrive(chassisSpeeds);
    }

    /**
     * Drives the swerve with the given velocities, relative to the field.
     *
     * @param translation the target x and y velocities in mps
     * @param rotation    the target theta velocity in radians per second
     */
    void fieldRelativeDrive(Translation2d translation, Rotation2d rotation) {
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(),
                translation.getY(),
                rotation.getRadians(),
                getHeading()
        );
        selfRelativeDrive(chassisSpeeds);
    }

    private void selfRelativeDrive(ChassisSpeeds chassisSpeeds) {
        if(isStill(chassisSpeeds)) {
            stop();
            return;
        }
        SwerveModuleState[] swerveModuleStates = SwerveConstants.KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        setTargetModuleStates(swerveModuleStates);
    }

    /**
     * Stops the swerve's motors.
     */
    public void stop() {
        for(SwerveModule module : SwerveConstants.SWERVE_MODULES)
            module.stop();
    }

    private void setTargetModuleStates(SwerveModuleState[] swerveModuleStates) {
        for(int i = 0; i < 4; i++)
            SwerveConstants.SWERVE_MODULES[i].setTargetState(swerveModuleStates[i]);
    }

    public void zeroHeading() {
        setHeading(0);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }
    private Rotation2d getGyroHeading() {
        return Rotation2d.fromDegrees(SwerveConstants.gyro.getYaw());
    }

    public void setHeading(double yaw) {
        poseEstimator.resetPosition(getPose(), getGyroHeading());
    }

    private boolean isStill(ChassisSpeeds chassisSpeeds) {
        return
                Math.abs(chassisSpeeds.vxMetersPerSecond) < SwerveConstants.DEAD_BAND_DRIVE_DEADBAND_MPS &&
                        Math.abs(chassisSpeeds.vyMetersPerSecond) < SwerveConstants.DEAD_BAND_DRIVE_DEADBAND_MPS &&
                        Math.abs(chassisSpeeds.omegaRadiansPerSecond) < SwerveConstants.DEAD_BAND_DRIVE_DEADBAND_MPS;
    }

    @Override
    public void periodic() {
        poseEstimator.update(
                getGyroHeading(),
                SwerveConstants.SWERVE_MODULES[0].getCurrentState(),
                SwerveConstants.SWERVE_MODULES[1].getCurrentState(),
                SwerveConstants.SWERVE_MODULES[2].getCurrentState(),
                SwerveConstants.SWERVE_MODULES[3].getCurrentState()
        );
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp) {
        poseEstimator.addVisionMeasurement(pose, timestamp);
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> measurementStdDevs) {
        poseEstimator.addVisionMeasurement(pose, timestamp, measurementStdDevs);
        poseEstimator.setVisionMeasurementStdDevs(SwerveConstants.VISION_STD_DEVS);
    }

    private void putOnDashboard() {
        for(int i = 0; i < SwerveConstants.SWERVE_MODULES.length; i++) {
            SmartDashboard.putData(
                    "Swerve/" + SwerveModuleConstants.SwerveModules.fromId(i).name(),
                    SwerveConstants.SWERVE_MODULES[i]
            );
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Heading", () -> getHeading().getDegrees(), this::setHeading);
        builder.addDoubleProperty("x", () -> getPose().getX(), null);
        builder.addDoubleProperty("y", () -> getPose().getY(), null);
    }

    public void resetPose() {
        for(SwerveModule module : SwerveConstants.SWERVE_MODULES)
            module.zeroDriveEncoder();
        poseEstimator.resetPosition(new Pose2d(), getGyroHeading());
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }
}

