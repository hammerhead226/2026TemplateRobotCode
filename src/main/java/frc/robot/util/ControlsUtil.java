package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;

public class ControlsUtil {
    public static final double DEFAULT_DEADBAND = 0.1;

    public static Translation2d applyDeadband(Translation2d input) {
        return applyDeadband(input, DEFAULT_DEADBAND);
    }

    public static Translation2d applyDeadband(Translation2d input, double deadband) {
        if (input.getNorm() <= deadband) return Translation2d.kZero;
        return input;
    }

    public static double applyDeadband(double input) {
        return applyDeadband(input, DEFAULT_DEADBAND);
    }

    public static double applyDeadband(double input, double deadband) {
        if (Math.abs(input) <= deadband) return 0.0;
        return input;
    }

    public static Translation2d squareNorm(Translation2d input) {
        return input.times(input.getNorm());
    }

    public static double squareNorm(double input) {
        return input * input * Math.signum(input);
    }
}
