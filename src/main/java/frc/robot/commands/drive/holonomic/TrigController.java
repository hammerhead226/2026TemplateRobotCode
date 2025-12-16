package frc.robot.commands.drive.holonomic;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO.Fiducial;
import java.util.Optional;

public class TrigController implements DriveController {
    private final Drive drive;
    private final Vision vision;
    private final int cameraIndex;
    private final int tagId;

    private Pose3d tagPose3d;
    private double lastOkTx;
    private double lastOkTy;

    private PIDPoseController pidPoseController;

    /**
     *
     * @param drive the drive subsystem.
     * @param vision the vision subsystem.
     * @param cameraIndex the index of the camera's VisionIO inside the vision subsystem's VisionIO[] io array. See the constructor for vision.
     * @param tagId the AprilTag id to base alignment off of.
     * @param tagToRobotIdeal the Transform2d that moves the tag's pose to the pose the robot will align to.
     *                          Ex: if the tag is facing east, x=2, y=1, rot=235° would cause the robot to align
     *                          2 meters east, 1 meter north of the tag, facing southwest.
     */
    public TrigController(Drive drive, Vision vision, int cameraIndex, int tagId, Transform2d tagToRobotIdeal) {
        this.drive = drive;
        this.vision = vision;
        this.cameraIndex = cameraIndex;
        this.tagId = tagId;
        tagPose3d = VisionConstants.aprilTagLayout.getTagPose(tagId).get();

        // tag space, tag center being (0,0) and the tag facing out to 0°
        Pose2d targetPoseTagSpace = Pose2d.kZero.transformBy(tagToRobotIdeal);
        pidPoseController = new PIDPoseController(drive, this::robotTagSpace, () -> targetPoseTagSpace);
    }

    @Override
    public void reset() {
        pidPoseController.reset();
    }

    @Override
    public ChassisSpeeds getSpeeds() {
        return pidPoseController.getSpeeds();
    }

    public Pose2d robotTagSpace() {
        // get tx and ty from camera
        Optional<Fiducial> fiducal = vision.getFiducial(cameraIndex, tagId);
        if (fiducal.isPresent()) {
            lastOkTx = fiducal.get().tx();
            lastOkTy = fiducal.get().ty();
        }

        Translation2d targetRobotSpace = targetRobotSpace(
                Math.toRadians(lastOkTx), Math.toRadians(lastOkTy), tagPose3d.getZ(), vision, cameraIndex);

        // raw gyro position used to continue to avoid potentially noisy vision measurements
        Transform2d robotToTagFieldSpace = new Transform2d(
                targetRobotSpace, tagPose3d.getRotation().toRotation2d().minus(drive.getRawGyroRotation()));
        // return to tag space by using the tag is the origin
        return Pose2d.kZero.transformBy(robotToTagFieldSpace.inverse());
    }

    public static Translation2d targetRobotSpace(
            double txRadians, double tyRadians, double targetZ, Vision vision, int cameraIndex) {
        return targetRobotSpace(txRadians, tyRadians, targetZ, vision.getRobotToCamera(cameraIndex));
    }

    // x is forward from the direction the robot faces
    // y is left from the direction the robot faces
    public static Translation2d targetRobotSpace(
            double txRadians, double tyRadians, double targetZ, Transform3d robotToCamera) {
        /*
         * projection center to projected point
         * <xi,yi,zi>                       in pixel image coordinates
         * = <zi*tan(tx),zi*tan(ty),zi>     tan(tx) = xi/zi, tan(ty) = yi/zi
         * ~ <tan(tx),tan(ty),1>            divide out the unknown zi
         *
         * pixel image coordinates to physical camera space
         * <right, down, forwards> = <+x,+y,+z>
         * <forwards, left, up> = <+z,-x,-y>
         *
         * <1,-tan(tx),-tan(ty)>
         *
         * scale by Δz/-tan(ty) to find camera to target
         * <-Δz/tan(ty),Δz*tan(tx)/tan(ty),Δz>
         */
        double deltaZ = targetZ - robotToCamera.getZ();
        Translation3d cameraToTarget = new Translation3d(
                -deltaZ / Math.tan(tyRadians), deltaZ * Math.tan(txRadians) / Math.tan(tyRadians), deltaZ);
        return Pose3d.kZero
                .plus(robotToCamera)
                .plus(new Transform3d(cameraToTarget, Rotation3d.kZero))
                .toPose2d()
                .getTranslation();
    }
}
