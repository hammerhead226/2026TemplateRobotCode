package frc.robot.util;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;

public record Translation2dController(ProfiledPIDController xController, ProfiledPIDController yController) {
    public Translation2dController() {
        this(new ProfiledPIDController(0, 0, 0, null), new ProfiledPIDController(0, 0, 0, null));
    }

    public Translation2d calculate(double xMeasurement, double yMeasurement) {
        return new Translation2d(xController.calculate(xMeasurement), yController.calculate(yMeasurement));
    }

    public Translation2d calculate(double xMeasurement, double xGoal, double yMeasurement, double yGoal) {
        return new Translation2d(
                xController.calculate(xMeasurement, xGoal), yController.calculate(yMeasurement, yGoal));
    }
}
