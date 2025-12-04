package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.vision.Vision;

public class TrigLocalizationUtil {
    public static Transform2d robotToTarget(
            double txRadians, double tyRadians, Transform3d robotToCamera, double targetZ) {
        // trig solve for the distance and yaw from camera to target when projected onto the xy plane (the ground)
        double groundToTargetRadians = robotToCamera.getRotation().getY() + tyRadians;

        // only accurate with large difference between camera z and target z
        double cameraToTargetMeters = (targetZ - robotToCamera.getZ()) / Math.tan(groundToTargetRadians);
        if (Double.isInfinite(cameraToTargetMeters)) {
            // angle is 0, target distance can't be determined by this method
            cameraToTargetMeters = Double.MAX_VALUE;
        }

        // tx is negated to convert to CWW+
        Rotation2d projectedGroundAngle = new Rotation2d(Math.tan(-txRadians), Math.cos(groundToTargetRadians));
        Transform2d cameraToTarget =
                new Transform2d(new Translation2d(cameraToTargetMeters, projectedGroundAngle), projectedGroundAngle);

        // obtain robot to target by calculating robot to camera to target, also projected onto the xy plane
        Transform2d robotToCamera2d = new Transform2d(
                robotToCamera.getTranslation().toTranslation2d(),
                robotToCamera.getRotation().toRotation2d());
        Transform2d robotToTargetEstimate = robotToCamera2d.plus(cameraToTarget);

        return robotToTargetEstimate;
    }

    public static Transform2d robotToTarget(
            double txRadians, double tyRadians, Vision vision, int cameraIndex, double targetZ) {
        return robotToTarget(txRadians, tyRadians, vision.getRobotToCamera(cameraIndex), targetZ);
    }
}
