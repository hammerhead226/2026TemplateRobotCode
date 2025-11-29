package frc.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.FieldMirroring;
import java.util.function.Supplier;

// TODO: extract angle logic into its own drive controller class
@Deprecated
public class JoystickDriveAtAngle extends Command {
    private Drive drive;
    private Supplier<Translation2d> translationSupplier;
    private Supplier<Rotation2d> rotationSupplier;

    private static final double ANGLE_KP = 5.0;
    private static final double ANGLE_KD = 0.4;
    private static final double ANGLE_MAX_VELOCITY = 8.0;
    private static final double ANGLE_MAX_ACCELERATION = 20.0;

    // Create PID controller
    private ProfiledPIDController angleController;

    /**
     * Field relative drive command using joystick for linear control and PID for angular control.
     * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
     * absolute rotation with a joystick.
     */
    public JoystickDriveAtAngle(
            Drive drive, Supplier<Translation2d> translationSupplier, Supplier<Rotation2d> rotationSupplier) {
        addRequirements(drive);
        this.drive = drive;
        this.translationSupplier = translationSupplier;
        this.rotationSupplier = rotationSupplier;
        angleController = new ProfiledPIDController(
                ANGLE_KP, 0.0, ANGLE_KD, new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
        angleController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        angleController.reset(drive.getRotation().getRadians());
    }

    @Override
    public void execute() {
        // Calculate angular speed
        double omega = angleController.calculate(
                drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

        // Convert to field relative speeds & send command
        ChassisSpeeds speeds = new ChassisSpeeds(
                translationSupplier.get().getX() * drive.getMaxLinearSpeedMetersPerSec(),
                translationSupplier.get().getY() * drive.getMaxLinearSpeedMetersPerSec(),
                omega);
        drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds, drive.getRotation().plus(FieldMirroring.driverStationFacing())));
    }
}
