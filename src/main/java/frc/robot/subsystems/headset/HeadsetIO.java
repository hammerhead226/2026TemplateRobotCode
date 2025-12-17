package frc.robot.subsystems.headset;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import gg.questnav.questnav.PoseFrame;
import org.littletonrobotics.junction.AutoLog;

public interface HeadsetIO {
    @AutoLog
    public static class HeadsetIOInputs {
        public boolean isConnected = false; // give error if this is false and don't accept estimations to the averager
        public int batteryPercent; // give error if this is too low
        public boolean tracking; // error if false
        public int frameCount; // error if it gets too low and don't accept estimations to the averager
        public int trackingLostCount; // log this
        public double latency; // error if it gets too high and don't accept estimations to the averager
        public PoseFrame[] poseFrames = new PoseFrame[0];
        public Transform3d robotToHeadset = new Transform3d();
    }

    public default void updateInputs(HeadsetIOInputs inputs) {}

    public default void commandPeriodic() {}

    public default void setPose(Pose3d pose3d) {}
}
