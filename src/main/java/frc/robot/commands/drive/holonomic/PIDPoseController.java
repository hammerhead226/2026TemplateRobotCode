package frc.robot.commands.drive.holonomic;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

// TODO: test
public class PIDPoseController implements DriveController {
    private final Drive drive;
    private final Supplier<Pose2d> estimatedPoseSupplier;
    private final Supplier<Pose2d> targetPoseSupplier;

    // TODO: add units to these constant names. Consider creating something like the PathConstraints object so it can be
    // passed in at the robotcontainer/command level and be flexible for the user to choose depending on what is being
    // accomplished
    // Maybe we could have preset pathconstraint objects in a constants file like "FastApproachConstraints",
    // "SlowApproachConstraints" etc
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

    @Override
    public void reset() {
        Pose2d estimatedPose = estimatedPoseSupplier.get();
        ChassisSpeeds fieldRelativeSpeeds =
                ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), estimatedPose.getRotation());

        xController.reset(estimatedPose.getX(), fieldRelativeSpeeds.vxMetersPerSecond);
        yController.reset(estimatedPose.getY(), fieldRelativeSpeeds.vyMetersPerSecond);
        angleController.reset(estimatedPose.getRotation().getRadians(), fieldRelativeSpeeds.omegaRadiansPerSecond);
    }

    @Override
    public ChassisSpeeds getSpeeds() {
        Pose2d targetPose = targetPoseSupplier.get();
        xController.setGoal(targetPose.getX());
        yController.setGoal(targetPose.getY());
        angleController.setGoal(targetPose.getRotation().getRadians());

        Pose2d estimatedPose = estimatedPoseSupplier.get();
        // TODO this is a hardcoded tagID, pass in to constructor
        // We may also want to be able to PID to a pose without an apriltag, ie if we're using global positioning
        Pose2d tagPose = VisionConstants.aprilTagLayout.getTagPose(13).get().toPose2d();
        Logger.recordOutput(
                "PIDPoseController/targetPose", tagPose.transformBy(new Transform2d(Pose2d.kZero, targetPose)));
        Logger.recordOutput(
                "PIDPoseController/estimatedPose", tagPose.transformBy(new Transform2d(Pose2d.kZero, estimatedPose)));

        return ChassisSpeeds.fromFieldRelativeSpeeds(
                new ChassisSpeeds(
                        xController.calculate(estimatedPose.getX()),
                        yController.calculate(estimatedPose.getY()),
                        angleController.calculate(estimatedPose.getRotation().getRadians())),
                estimatedPose.getRotation());
    }
}
