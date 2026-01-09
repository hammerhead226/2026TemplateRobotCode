// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.SimConstants;
import frc.robot.constants.SubsystemConstants.FlywheelConstants;
import frc.robot.util.LoggedTunableNumber;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO flywheel;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private SimpleMotorFeedforward ffModel;
  private final SysIdRoutine sysId;

  private static final String flywheelName = FlywheelConstants.FLYWHEEL_STRING;

  private static final LoggedTunableNumber kV = new LoggedTunableNumber(flywheelName + "/kV", 1);
  private static final LoggedTunableNumber kS = new LoggedTunableNumber(flywheelName + "/kS", 1);
  private static final LoggedTunableNumber kA = new LoggedTunableNumber(flywheelName + "/kA", 1);

  /** Creates a new Flywheel. */
  public Flywheel(FlywheelIO flywheel) {
    this.flywheel = flywheel;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (SimConstants.currentMode) {
      case REAL:
        kV.initDefault(0);
        kS.initDefault(0);
        kA.initDefault(0);
        break;

      case REPLAY:
        kV.initDefault(0);
        kS.initDefault(0);
        kA.initDefault(0);
        break;
      case SIM:
        kV.initDefault(0);
        kS.initDefault(0);
        kA.initDefault(0);
        break;
      default:
        kV.initDefault(0);
        kS.initDefault(0);
        kA.initDefault(0);
        break;

    }

    // Configure SysId
    sysId = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            null,
            null,
            (state) -> Logger.recordOutput("Flywheel/SysIdState", state.toString())),
        new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));

    ffModel = new SimpleMotorFeedforward(kS.get(), kV.get(), kA.get());
    updateTunableNumbers();
  }

  @Override
  public void periodic() {
    flywheel.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);

    updateTunableNumbers();
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    flywheel.setVoltage(volts);
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    flywheel.setVelocity(
        velocityRadPerSec,
        ffModel
            .calculate(velocityRadPerSec));

    // Log flywheel setpoint
    Logger.recordOutput("Flywheel/SetpointRPM", velocityRPM);
  }

  public Command runVoltsCommmand(double volts) {

    return new InstantCommand(() -> runVolts(volts), this);
  }

  public Command runVelocityCommand(double velocityRPM) {

    return new InstantCommand(() -> runVelocity(velocityRPM), this);
  }

  public Command flywheelStop() {
    return new InstantCommand(() -> stop(), this);
  }

  /** Stops the flywheel. */
  public void stop() {
    flywheel.stop();
  }

  public Command stopCommand() {
    return new InstantCommand(() -> stop(), this);
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /** Returns the current velocity in RPM. */
  @AutoLogOutput
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  /** Returns the current velocity in radians per second. */
  public double getCharacterizationVelocity() {
    return inputs.velocityRadPerSec;
  }

  public void updateTunableNumbers() {
    if (kV.hasChanged(hashCode()) || kA.hasChanged(hashCode()) || kS.hasChanged(hashCode())) {
      ffModel = new SimpleMotorFeedforward(kS.get(), kV.get(), kA.get());
    }
  }
}
