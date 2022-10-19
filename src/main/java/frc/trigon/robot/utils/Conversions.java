package frc.trigon.robot.utils;

public class Conversions {

    public static final int DEGREES_PER_REVOLUTIONS = 360;
    public static final int MAG_TICKS_PER_REVOLUTION = 4096;
    public static final int SECONDS_PER_MINUTE = 60;
    public static final int HUNDRED_MS_PER_SECOND = 10;
    public static final int FALCON_TICKS_PER_REVOLUTIONS = 2048;

    public static double degreesToMagTicks(double degrees) {
        return degreesToRevolutions(degrees) * MAG_TICKS_PER_REVOLUTION;
    }

    public static double degreesToRevolutions(double degrees) {
        return degrees / DEGREES_PER_REVOLUTIONS;
    }

    public static double falconTicksPer100MsToRpm(double ticks) {
        return falconTicksToRevolutions(vPer100MsToVPerMinute(ticks));
    }

    public static double falconTicksToRevolutions(double ticks) {
        return ticks / FALCON_TICKS_PER_REVOLUTIONS;
    }

    public static double vPer100MsToVPerSecond(double v) {
        return v * HUNDRED_MS_PER_SECOND;
    }

    public static double vPerSecondToVPerMinute(double v) {
        return v * SECONDS_PER_MINUTE;
    }

    public static double vPer100MsToVPerMinute(double v) {
        return vPerSecondToVPerMinute(vPer100MsToVPerSecond(v));
    }

    public static double magTicksToDegrees(double ticks) {
        return revolutionsToDegrees(magTicksToRevolutions(ticks));
    }

    public static double revolutionsToDegrees(double revolutions) {
        return revolutions * DEGREES_PER_REVOLUTIONS;
    }

    public static double magTicksToRevolutions(double ticks) {
        return ticks / MAG_TICKS_PER_REVOLUTION;
    }

    public static double revolutionsToMeters(double revolutions, double radius) {
        return revolutions * (2 * Math.PI * radius);
    }

    public static double systemPositionToEncoderPosition(double systemPosition, double gearRatio) {
        return systemPosition * gearRatio;
    }

    public static double encoderPositionToSystemPosition(double encoderPosition, double gearRatio) {
        return encoderPosition / gearRatio;
    }

    public static double vPerMinuteToVPerSecond(double v) {
        return v / SECONDS_PER_MINUTE;
    }
}
