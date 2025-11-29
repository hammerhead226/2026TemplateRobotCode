package frc.robot.subsystems.distance;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorRangeIO {
  @AutoLog
  public static class ElevatorRangeInputs {
    double elevatorPositionInch = 0;
    double elevatorVelocityInchesPerSecond = 0;
    double currentAmps = 0;
    double appliedVolts = 0;
    double positionSetpointInch = 0;
  }

  public default void updateInputs(ElevatorRangeInputs inputs) {}

  public default void runCharacterization(double volts) {}

  public default void setPositionSetpoint(double position, double ffVolts) {}

  public default void stop() {}

  public default void setVoltage(double volts) {}

  public default void configurePID(double kP, double kI, double kD) {}
}
