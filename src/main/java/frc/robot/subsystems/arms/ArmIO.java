package frc.robot.subsystems.arms;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public double velocityDegsPerSec = 0;
    public double positionDegs = 0;
    public double currentAmps = 0;
    public double appliedVolts = 0;
    public double positionSetpointDegs;

    public boolean gyroConnected = false;
    public double pitch = 0;
  }

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setBrakeMode(boolean bool) {}

  public default void setPositionSetpointDegs(double positionDegs, double ffVolts) {}

  public default void setVoltage(double volts) {}

  public default void stop() {}

  public default void configurePID(double kP, double kI, double kD) {}
}
