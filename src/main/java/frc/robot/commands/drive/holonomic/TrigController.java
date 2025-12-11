package frc.robot.commands.drive.holonomic;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO.Fiducial;
import frc.robot.util.TrigLocalizationUtil;
import java.util.Optional;

public class TrigController implements DriveController {
    private final Vision vision;
    private final int cameraIndex;
    private final int tagIndex;

    private Pose3d tagPose3d;
    private double lastOkTx;
    private double lastOkTy;

    private PIDPoseController pidPoseController;

    public TrigController(Drive drive, Vision vision, int cameraIndex, int tagIndex, Transform2d robotToTargetIdeal) {
        this.vision = vision;
        this.cameraIndex = cameraIndex;
        this.tagIndex = tagIndex;
        tagPose3d = VisionConstants.aprilTagLayout.getTagPose(tagIndex).get();

        // in this coordinate space consider the tag to be Pose2d.kZero
        Pose2d targetPose = Pose2d.kZero.transformBy(robotToTargetIdeal.inverse());
        pidPoseController = new PIDPoseController(
                drive, () -> Pose2d.kZero.transformBy(robotToTarget().inverse()), () -> targetPose);
    }

    @Override
    public void reset() {
        pidPoseController.reset();
    }

    @Override
    public ChassisSpeeds getSpeeds() {
        return pidPoseController.getSpeeds();
    }

    public Transform2d robotToTarget() {
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
