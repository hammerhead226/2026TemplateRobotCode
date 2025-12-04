package frc.robot.commands.drive.holonomic;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

// TODO: test
public class PIDPoseController {
    private final Drive drive;
    private final Supplier<Pose2d> estimatedPoseSupplier;
    private final Supplier<Pose2d> targetPoseSupplier;

    // TODO: make this suck less and find reasonable default values
    private static final double X_KP = 5.0;
    private static final double X_KD = 0.4;
    private static final double X_MAX_VELOCITY = 8.0;
    private static final double X_MAX_ACCELERATION = 20.0;

    private static final double Y_KP = 5.0;
    private static final double Y_KD = 0.4;
    private static final double Y_MAX_VELOCITY = 8.0;
    private static final double Y_MAX_ACCELERATION = 20.0;

    private static final double ANGLE_KP = 5.0;
    private static final double ANGLE_KD = 0.4;
    private static final double ANGLE_MAX_VELOCITY = 8.0;
    private static final double ANGLE_MAX_ACCELERATION = 20.0;

    private ProfiledPIDController xController;
    private ProfiledPIDController yController;
    private ProfiledPIDController angleController;

    public PIDPoseController(Drive drive, Supplier<Pose2d> estimatedPoseSupplier, Supplier<Pose2d> targetPoseSupplier) {
        this.drive = drive;
        this.estimatedPoseSupplier = estimatedPoseSupplier;
        this.targetPoseSupplier = targetPoseSupplier;

        xController = new ProfiledPIDController(
                X_KP, 0.0, X_KD, new TrapezoidProfile.Constraints(X_MAX_VELOCITY, X_MAX_ACCELERATION));
        yController = new ProfiledPIDController(
                Y_KP, 0.0, Y_KD, new TrapezoidProfile.Constraints(Y_MAX_VELOCITY, Y_MAX_ACCELERATION));
        angleController = new ProfiledPIDController(
                ANGLE_KP, 0.0, ANGLE_KD, new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
        angleController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void reset() {
        Pose2d estimatedPose = estimatedPoseSupplier.get();
        xController.reset(estimatedPose.getX(), drive.getChassisSpeeds().vxMetersPerSecond);
        yController.reset(estimatedPose.getY(), drive.getChassisSpeeds().vyMetersPerSecond);
        angleController.reset(estimatedPose.getRotation().getRadians(), drive.getChassisSpeeds().omegaRadiansPerSecond);
    }

    public ChassisSpeeds getSpeeds() {
        Pose2d targetPose = targetPoseSupplier.get();
        xController.setGoal(targetPose.getX());
        yController.setGoal(targetPose.getY());
        angleController.setGoal(targetPose.getRotation().getRadians());

        Pose2d estimatedPose = estimatedPoseSupplier.get();
        return new ChassisSpeeds(
                xController.calculate(estimatedPose.getX()) * drive.getMaxLinearSpeedMetersPerSec(),
                yController.calculate(estimatedPose.getY()) * drive.getMaxLinearSpeedMetersPerSec(),
                angleController.calculate(estimatedPose.getRotation().getRadians())
                        * drive.getMaxAngularSpeedRadPerSec());
    }
}
