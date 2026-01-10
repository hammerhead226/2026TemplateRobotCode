package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        double elevatorPositionInch = 0;
        double elevatorVelocityInchesPerSecond = 0;
        double leaderStatorCurrentAmps = 0;
        double leaderSupplyCurrentAmps = 0;
        double followerStatorCurrentAmps = 0;
        double followerSupplyCurrentAmps = 0;
        double appliedVolts = 0;
        double positionSetpointInch = 0;
        double CANrangeDistanceInches;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void runCharacterization(double volts) {}

    public default void setPositionSetpoint(double position, double ffVolts) {}

    public default void stop() {}

    public default void setVoltage(double volts) {}

    public default void zeroElevator() {}

    public default void configurePID(double kP, double kI, double kD) {}

    public default void setBrakeMode(boolean brake) {}
}
