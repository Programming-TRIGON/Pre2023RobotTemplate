package frc.trigon.robot.components;

public class XboxController extends edu.wpi.first.wpilibj.XboxController {
    private boolean square = false;
    private double deadband = 0;

    /**
     * Construct an instance of a controller.
     *
     * @param port The port index on the Driver Station that the controller is plugged into.
     */
    public XboxController(int port) {
        super(port);
    }

    /**
     * Construct an instance of a controller.
     *
     * @param port     the port index on the Driver Station that the controller is plugged into
     * @param square   whether to square the input values
     * @param deadband the deadband for the controller
     */
    public XboxController(int port, boolean square, double deadband) {
        super(port);
        this.square = square;
        this.deadband = deadband;
    }

    /**
     * Determines whether the controller is in square mode,
     * which will take the raw value and square it for added precision in the lower values.
     *
     * @param square whether to square the raw values
     */
    public void setSquare(boolean square) {
        this.square = square;
    }

    /**
     * Sets the deadband for the controller, which will ignore any values within the deadband.
     *
     * @param deadband the deadband, between 0 and 1
     */
    public void setDeadband(double deadband) {
        this.deadband = deadband;
    }

    @Override
    public double getLeftX() {
        return calculateValue(super.getLeftX());
    }

    @Override
    public double getRightX() {
        return calculateValue(super.getRightX());
    }

    @Override
    public double getLeftY() {
        return calculateValue(-super.getLeftY());
    }

    @Override
    public double getRightY() {
        return calculateValue(-super.getRightY());
    }

    public edu.wpi.first.wpilibj2.command.button.Button getLeftBumperBtn() {
        return new edu.wpi.first.wpilibj2.command.button.Button(this::getLeftBumper);
    }

    public edu.wpi.first.wpilibj2.command.button.Button getRightBumperBtn() {
        return new edu.wpi.first.wpilibj2.command.button.Button(this::getRightBumper);
    }

    public edu.wpi.first.wpilibj2.command.button.Button getLeftStickBtn() {
        return new edu.wpi.first.wpilibj2.command.button.Button(this::getLeftStickButton);
    }

    public edu.wpi.first.wpilibj2.command.button.Button getRightStickBtn() {
        return new edu.wpi.first.wpilibj2.command.button.Button(this::getRightStickButton);
    }

    public edu.wpi.first.wpilibj2.command.button.Button getABtn() {
        return new edu.wpi.first.wpilibj2.command.button.Button(this::getAButton);
    }

    public edu.wpi.first.wpilibj2.command.button.Button getBBtn() {
        return new edu.wpi.first.wpilibj2.command.button.Button(this::getBButton);
    }

    public edu.wpi.first.wpilibj2.command.button.Button getXBtn() {
        return new edu.wpi.first.wpilibj2.command.button.Button(this::getXButton);
    }

    public edu.wpi.first.wpilibj2.command.button.Button getYBtn() {
        return new edu.wpi.first.wpilibj2.command.button.Button(this::getYButton);
    }

    public edu.wpi.first.wpilibj2.command.button.Button getBackBtn() {
        return new edu.wpi.first.wpilibj2.command.button.Button(this::getBackButton);
    }

    public edu.wpi.first.wpilibj2.command.button.Button getStartBtn() {
        return new edu.wpi.first.wpilibj2.command.button.Button(this::getStartButton);
    }

    public edu.wpi.first.wpilibj2.command.button.Button getRawButtonBtn(int button) {
        return new edu.wpi.first.wpilibj2.command.button.Button(() -> getRawButton(button));
    }

    private double calculateValue(double value) {
        value = Math.pow(value, 2) * Math.signum(value);
        if(Math.abs(value) < deadband)
            return 0;
        return value;
    }
}
