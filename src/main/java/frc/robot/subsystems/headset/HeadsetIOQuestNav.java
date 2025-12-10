package frc.robot.subsystems.headset;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class HeadsetIOQuestNav implements HeadsetIO {
    public QuestNav headset;
    Transform3d questRobotPose;
    Pose3d estimatedRobotPose;
    Rotation3d estimatedRobotRotation;

    public HeadsetIOQuestNav() {
        headset = new QuestNav();
        questRobotPose = new Transform3d(); // add quest's x,y and rotational offset from robot
        estimatedRobotPose = new Pose3d();
        estimatedRobotRotation = new Rotation3d();
    }

    @Override
    public void updateInputs(HeadsetIOInputs inputs) {
        inputs.isConnected = headset.isConnected();
        inputs.latency = headset.getLatency();
        inputs.batteryPercent = headset.getBatteryPercent().getAsInt();
        inputs.frameCount = headset.getFrameCount().getAsInt();
        inputs.tracking = headset.isTracking();
        inputs.trackingLostCount = headset.getTrackingLostCounter().getAsInt();

        PoseFrame[] poseFrames = headset.getAllUnreadPoseFrames();

        if (poseFrames.length > 0) {
            // Get the most recent Quest pose
            Pose3d questPose = poseFrames[poseFrames.length - 1].questPose3d();

            // Transform by the mount pose to get your robot pose
            estimatedRobotPose = questPose.transformBy(questRobotPose.inverse());
            inputs.estimatedRobotPose = estimatedRobotPose;
            inputs.estimatedRotation =
                    questPose.transformBy(questRobotPose.inverse()).getRotation();
        }
    }

    @Override
    public void commandPeriodic() {
        headset.commandPeriodic();
    }

    @Override
    public void zeroCommand() {
        headset.setPose(new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)));
    }
}
