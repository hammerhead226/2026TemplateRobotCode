package frc.robot.commands.drive.holonomic;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.ControlsUtil;
import frc.robot.util.FieldMirroring;
import java.util.function.DoubleSupplier;

public class JoystickController implements DriveController {
    Drive drive;
    DoubleSupplier xSupplier;
    DoubleSupplier ySupplier;
    DoubleSupplier rotationSupplier;

    public JoystickController(
            Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotationSupplier) {
        this.drive = drive;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotationSupplier = rotationSupplier;
    }

    @Override
    public ChassisSpeeds getSpeeds() {
        return JoystickController.getSpeeds(
                drive, xSupplier.getAsDouble(), ySupplier.getAsDouble(), rotationSupplier.getAsDouble());
    }

    public static ChassisSpeeds getSpeeds(Drive drive, double x, double y, double rotation) {
        // deadband to ignore tiny inputs
        // square to make easier to control
        Translation2d velocity = ControlsUtil.squareNorm(ControlsUtil.applyDeadband(new Translation2d(-y, -x)));
        double omega = ControlsUtil.squareNorm(ControlsUtil.applyDeadband(-rotation));

        // rotate velocity based on driver station
        velocity = velocity.rotateBy(FieldMirroring.driverStationFacing());

        // TODO Ideally we should add a scalar to adjust the driver's max speed (which may be different than the max
        // speed we run in auto)
        // scale to drive's max speed
        ChassisSpeeds speeds = new ChassisSpeeds(
                velocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                velocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                omega * drive.getMaxAngularSpeedRadPerSec());

        // make robot relative
        return ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation());
    }
}
