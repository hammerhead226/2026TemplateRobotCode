package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.vision.Vision;

public class TrigLocalization {
    public static Transform2d robotToTarget(double txRadians, double tyRadians, Transform3d robotToCamera, double targetY) {
        // trig solve for the distance and yaw from camera to target when projected onto the xy plane (the ground)
        double groundToTargetRadians = robotToCamera.getRotation().getY()+tyRadians;
        double cameraToTargetMeters = (targetY-robotToCamera.getY()) / Math.tan(groundToTargetRadians);
        Rotation2d projectedGroundAngle = new Rotation2d(
            Math.tan(txRadians), 
            Math.cos(groundToTargetRadians)
        );
        Transform2d cameraToTarget = new Transform2d(
            new Translation2d(cameraToTargetMeters,projectedGroundAngle),
            projectedGroundAngle
        );

        // obtain robot to target by calculating robot to camera to target, also projected onto the xy plane
        Transform2d robotToCamera2d = new Transform2d(
            robotToCamera.getTranslation().toTranslation2d(), 
            robotToCamera.getRotation().toRotation2d()
        );
        Transform2d robotToTargetEstimate = robotToCamera2d.plus(cameraToTarget);

        return robotToTargetEstimate;
    }

    public static Transform2d robotToTarget(double txRadians, double tyRadians, Vision vision, int cameraIndex, double targetY) {
        return robotToTarget(txRadians, tyRadians, vision.getRobotToCamera(cameraIndex), targetY);
    }
}