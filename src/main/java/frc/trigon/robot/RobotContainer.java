package frc.trigon.robot;

import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.trigon.robot.commands.UpdatePoseFromTagCommand;
import frc.trigon.robot.components.VisionManager;
import frc.trigon.robot.components.XboxController;
import frc.trigon.robot.subsystems.swerve.OpenLoopFieldRelativeSupplierDrive;
import frc.trigon.robot.subsystems.swerve.Swerve;

public class RobotContainer {
    public final Field2d smartDashboardField;
    XboxController controller;

    OpenLoopFieldRelativeSupplierDrive fieldRelativeXboxDrive;
    OpenLoopFieldRelativeSupplierDrive slowFieldRelativeXboxDrive;

    public RobotContainer() {
        smartDashboardField = new Field2d();
        SmartDashboard.putData("Field", smartDashboardField);

        initComponents();
        initCommands();
        bindDefaultCommands();
        bindControllerCommands();
        bindStandaloneCommands();

        postToDashboard();

        LiveWindow.disableAllTelemetry();
    }

    private static void bindStandaloneCommands() {
        new UpdatePoseFromTagCommand(VisionManager.getInstance().camera, VisionManager.getInstance().cameraPose).schedule();
    }

    private void postToDashboard() {
        SmartDashboard.putData(Swerve.getInstance());
    }

    private void initCommands() {
        fieldRelativeXboxDrive = new OpenLoopFieldRelativeSupplierDrive(
                () -> controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> -controller.getRightX()
        );
        slowFieldRelativeXboxDrive = new OpenLoopFieldRelativeSupplierDrive(
                () -> controller.getLeftY() / 2,
                () -> -controller.getLeftX() / 2,
                () -> -controller.getRightX() / 2
        );
    }

    private void bindDefaultCommands() {
        Swerve.getInstance().setDefaultCommand(fieldRelativeXboxDrive);
    }

    private void bindControllerCommands() {
        controller.getYBtn().whenPressed(Swerve.getInstance()::resetPose);
        controller.getRightBumperBtn().whenPressed(slowFieldRelativeXboxDrive).whenReleased(slowFieldRelativeXboxDrive::cancel);
    }

    private void initComponents() {
        controller = new XboxController(0, true, 0.03);
    }
}
