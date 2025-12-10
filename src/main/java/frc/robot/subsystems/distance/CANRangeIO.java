package frc.robot.subsystems.distance;

import org.littletonrobotics.junction.AutoLog;

public interface CANRangeIO {
    @AutoLog
    public static class CANRangeIOInputs {
        public double distanceInches = 0.0;
        public boolean connected = false;
        public double distanceStd = 0.0;
    }

    public default void updateInputs(CANRangeIOInputs inputs) {}

    public default double getDistance() {
        return 0.0;
    }

    public default boolean isInRange() {
        return false;
    }
}
