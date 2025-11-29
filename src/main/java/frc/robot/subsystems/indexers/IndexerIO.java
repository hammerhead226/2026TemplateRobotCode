package frc.robot.subsystems.indexers;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    public static class IndexerIOInputs {
        double indexerPositionInch = 0;
        double indexerVelocityInchesPerSecond = 0;
        double currentAmps = 0;
        double appliedVolts = 0;
        double positionSetpointInch = 0;
    }

    default void updateInputs(IndexerIOInputs inputs) {}

    default void runCharacterization(double volts) {}

    default void stop() {}

    default void setPositionSetpoint(double positionDegs, double ffvolts) {}

    default void configurePID(double kP, double kI, double kD) {}
}
