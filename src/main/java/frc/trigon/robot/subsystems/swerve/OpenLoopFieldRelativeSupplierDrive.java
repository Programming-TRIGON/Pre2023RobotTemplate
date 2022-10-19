package frc.trigon.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

public class OpenLoopFieldRelativeSupplierDrive extends CommandBase {
    private final DoubleSupplier xPower;
    private final DoubleSupplier yPower;
    private final DoubleSupplier rotPower;
    private final Swerve swerve = Swerve.getInstance();

    /**
     * Drives the swerve relative to the robot.
     *
     * @param xPower   the forwards power.
     * @param yPower   the leftwards power.
     * @param rotPower the rotational power.
     */
    public OpenLoopFieldRelativeSupplierDrive(DoubleSupplier xPower, DoubleSupplier yPower, DoubleSupplier rotPower) {
        this.xPower = xPower;
        this.yPower = yPower;
        this.rotPower = rotPower;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        swerve.fieldRelativeDrive(
                new Translation2d(
                        xPower.getAsDouble() * SwerveConstants.MAX_SPEED_METERS_PER_SECOND,
                        yPower.getAsDouble() * SwerveConstants.MAX_SPEED_METERS_PER_SECOND
                ),
                new Rotation2d(rotPower.getAsDouble() * SwerveConstants.MAX_ROTATIONAL_SPEED_RADIANS_PER_SECOND)
        );
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }
}
