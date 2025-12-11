package frc.robot.subsystems.headset;

import java.util.OptionalDouble;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class HeadsetIOQuestNav implements HeadsetIO {
    public QuestNav questNav;
    Transform3d questRobotPose;
    Pose3d estimatedRobotPose;
    Rotation3d estimatedRobotRotation;

    public HeadsetIOQuestNav() {
        questNav = new QuestNav();
        questRobotPose = new Transform3d(); // add quest's x,y and rotational offset from robot
        estimatedRobotPose = new Pose3d();
        estimatedRobotRotation = new Rotation3d();
    }

    @Override
    public void updateInputs(HeadsetIOInputs inputs) {
        inputs.isConnected = questNav.isConnected();
        inputs.latency = questNav.getLatency();
        inputs.batteryPercent = questNav.getBatteryPercent().orElse(0);
        inputs.frameCount = questNav.getFrameCount().orElse(0);
        inputs.tracking = questNav.isTracking();
        inputs.trackingLostCount = questNav.getTrackingLostCounter().orElse(0);
        inputs.appTimestamp = questNav.getAppTimestamp().orElse(0);
        

        PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();

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
        questNav.commandPeriodic();
    }

    @Override
    public void setPose(Pose3d pose3d) {
        questNav.setPose(pose3d);
    }
}
