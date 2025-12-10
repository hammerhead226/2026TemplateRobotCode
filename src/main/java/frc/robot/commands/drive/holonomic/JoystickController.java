package frc.robot.commands.drive.holonomic;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.ControlsUtil;
import frc.robot.util.FieldMirroring;

public class JoystickController {
    public static ChassisSpeeds getSpeeds(Drive drive, double x, double y, double rotation) {
        // deadband to ignore tiny inputs
        // square to make easier to control
        Translation2d velocity = ControlsUtil.squareNorm(ControlsUtil.applyDeadband(new Translation2d(-y, -x)));
        double omega = ControlsUtil.squareNorm(ControlsUtil.applyDeadband(-rotation));

        // rotate velocity based on driver station
        velocity = velocity.rotateBy(FieldMirroring.driverStationFacing());

        // scale to drive's max speed
        ChassisSpeeds speeds = new ChassisSpeeds(
                velocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                velocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                omega * drive.getMaxAngularSpeedRadPerSec());

        // make field relative
        return ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation());
    }
}
