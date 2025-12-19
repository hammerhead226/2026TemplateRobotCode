package frc.robot.subsystems.headset;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import gg.questnav.questnav.QuestNav;

public class HeadsetIOQuestNav implements HeadsetIO {
    public QuestNav questNav;
    Transform3d robotToHeadset;
    Pose3d estimatedRobotPose;
    Rotation3d estimatedRobotRotation;

    public HeadsetIOQuestNav() {
        questNav = new QuestNav();
        robotToHeadset = new Transform3d(); // TODO add quest's x,y and rotational offset from robot (consider to use
        // VisionConstants.jav)
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
        inputs.poseFrames = questNav.getAllUnreadPoseFrames();
        inputs.robotToHeadset = this.robotToHeadset;
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
