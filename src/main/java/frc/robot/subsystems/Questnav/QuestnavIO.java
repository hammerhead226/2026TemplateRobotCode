package frc.robot.subsystems.Questnav;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.littletonrobotics.junction.AutoLog;

public interface QuestnavIO {
  @AutoLog
  public static class QuestnavIOInputs {
    public boolean isConnected =
        false; // give error if this is false and don't accept estimations to the averager
    public int batteryPercent; // give error if this is too low
    public boolean tracking; // error if false
    public int frameCount; // error if it gets too low and don't accept estimations to the averager
    public int trackingLostCount; // log this
    public double latency; // error if it gets too high and don't accept estimations to the averager
    public double[] appTimestamps = new double[] {}; // log this
    public Pose3d estimatedRobotPose;
    public Rotation3d estimatedRotation;
  }

  public default void updateInputs(QuestnavIOInputs inputs) {}

  public default void commandPeriodic() {}

  public default void zeroCommand() {}
}
