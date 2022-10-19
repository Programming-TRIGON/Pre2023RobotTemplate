package frc.trigon.robot;

import frc.trigon.robot.components.XboxController;
import frc.trigon.robot.subsystems.swerve.OpenLoopFieldRelativeSupplierDrive;
import frc.trigon.robot.subsystems.swerve.Swerve;

public class RobotContainer {
    XboxController controller;

    OpenLoopFieldRelativeSupplierDrive fieldRelativeXboxDrive;
    OpenLoopFieldRelativeSupplierDrive slowFieldRelativeXboxDrive;
    public RobotContainer() {
        initComponents();
        initCommands();
        bindDefaultCommands();
        bindControllerCommands();
    }

    private void initCommands() {
        fieldRelativeXboxDrive = new OpenLoopFieldRelativeSupplierDrive(
                () -> controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> -controller.getRightX()
        );
        slowFieldRelativeXboxDrive = new OpenLoopFieldRelativeSupplierDrive(
                () -> controller.getLeftY() /2,
                () -> -controller.getLeftX() /2,
                () -> -controller.getRightX() /2
        );
    }

    private void bindDefaultCommands() {
        Swerve.getInstance().setDefaultCommand(fieldRelativeXboxDrive);
    }

    private void bindControllerCommands(){
        controller.getYBtn().whenPressed(Swerve.getInstance()::zeroHeading);
        controller.getRightBumperBtn().whenPressed(slowFieldRelativeXboxDrive).whenReleased(slowFieldRelativeXboxDrive::cancel);
    }

    private void initComponents(){
        controller = new XboxController(0, true, 0.05);
    }


}
