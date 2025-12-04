package frc.robot.commands.drive.holonomic;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO.Fiducial;
import frc.robot.util.TrigLocalizationUtil;
import java.util.Optional;

// TODO: test
public class TrigController {
    private final Drive drive;
    private final Vision vision;
    private final int cameraIndex;
    private final int tagIndex;
    private final Transform2d robotToTargetIdeal;

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

    private Pose3d tagPose3d;

    private ProfiledPIDController xController;
    private ProfiledPIDController yController;
    private ProfiledPIDController angleController;
    private double lastOkTx;
    private double lastOkTy;

    public TrigController(Drive drive, Vision vision, int cameraIndex, int tagIndex, Transform2d robotToTargetIdeal) {
        this.drive = drive;
        this.vision = vision;
        this.cameraIndex = cameraIndex;
        this.tagIndex = tagIndex;
        this.robotToTargetIdeal = robotToTargetIdeal;

        tagPose3d = VisionConstants.aprilTagLayout.getTagPose(tagIndex).get();

        xController = new ProfiledPIDController(
                X_KP, 0.0, X_KD, new TrapezoidProfile.Constraints(X_MAX_VELOCITY, X_MAX_ACCELERATION));
        yController = new ProfiledPIDController(
                Y_KP, 0.0, Y_KD, new TrapezoidProfile.Constraints(Y_MAX_VELOCITY, Y_MAX_ACCELERATION));
        angleController = new ProfiledPIDController(
                ANGLE_KP, 0.0, ANGLE_KD, new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
        angleController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void reset() {
        Transform2d robotToTarget = robotToTarget();
        ChassisSpeeds tagRelativeSpeeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(drive.getChassisSpeeds(), robotToTarget.getRotation());
        xController.reset(robotToTarget.getX(), tagRelativeSpeeds.vxMetersPerSecond);
        yController.reset(robotToTarget.getY(), tagRelativeSpeeds.vyMetersPerSecond);
        angleController.reset(robotToTarget.getRotation().getRadians(), tagRelativeSpeeds.omegaRadiansPerSecond);

        xController.setGoal(robotToTargetIdeal.getX());
        yController.setGoal(robotToTargetIdeal.getY());
        angleController.setGoal(robotToTargetIdeal.getRotation().getRadians());
    }

    public ChassisSpeeds getSpeeds() {
        Transform2d robotToTarget = robotToTarget();

        return ChassisSpeeds.fromFieldRelativeSpeeds(
                new ChassisSpeeds(
                        xController.calculate(robotToTarget.getX()) * drive.getMaxLinearSpeedMetersPerSec(),
                        yController.calculate(robotToTarget.getY()) * drive.getMaxLinearSpeedMetersPerSec(),
                        angleController.calculate(robotToTarget.getRotation().getRadians())
                                * drive.getMaxAngularSpeedRadPerSec()),
                robotToTarget.getRotation());
    }

    private Transform2d robotToTarget() {
        // get tx and ty from camera
        Optional<Fiducial> fiducal = vision.getFiducial(cameraIndex, tagIndex);
        if (fiducal.isPresent()) {
            lastOkTx = fiducal.get().tx();
            lastOkTy = fiducal.get().ty();
        }

        return TrigLocalizationUtil.robotToTarget(
                Math.toRadians(lastOkTx), Math.toRadians(lastOkTy), vision, cameraIndex, tagPose3d.getY());
    }
}
