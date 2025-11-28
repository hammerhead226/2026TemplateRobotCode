package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.ControlsUtil;
import frc.robot.util.FieldMirroring;

public class JoystickControl {
    public static Translation2d getVelocity(double x, double y) {
        return ControlsUtil.squareNorm(
            ControlsUtil.applyDeadband(
                new Translation2d(-y, -x))).rotateBy(FieldMirroring.driverStationFacing());
    }

    public static double getOmega(double omega) {
        return ControlsUtil.squareNorm(ControlsUtil.applyDeadband(omega));
    }
}
