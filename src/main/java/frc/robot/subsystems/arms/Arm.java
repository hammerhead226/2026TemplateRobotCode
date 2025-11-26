// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arms;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SimConstants;
import frc.robot.constants.SubsystemConstants;
import frc.robot.constants.SubsystemConstants.ArmConstants.TuningConstants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

import frc.robot.constants.SubsystemConstants.ArmConstants;

public class Arm extends SubsystemBase {
  private static final String armName = ArmConstants.ARM_STRING;
  private final ArmIO arm;
  private final ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();


  private static LoggedTunableNumber kP = new LoggedTunableNumber(armName + "/kP");
  private static LoggedTunableNumber kG = new LoggedTunableNumber(armName + "/kG");
  private static LoggedTunableNumber kV = new LoggedTunableNumber(armName + "/kV");
  private static LoggedTunableNumber kA = new LoggedTunableNumber(armName + "/kA");
  private static LoggedTunableNumber kS = new LoggedTunableNumber(armName + "/kS");
  private static LoggedTunableNumber kI = new LoggedTunableNumber(armName + "/kI");
  private static LoggedTunableNumber kD = new LoggedTunableNumber(armName + "/kD");

  private static LoggedTunableNumber maxVelocityDegPerSec = new LoggedTunableNumber(armName + "/maxVelocityDegPerSec",
      ArmConstants.MAX_VELOCITY_DEG_PER_SEC);
  private static LoggedTunableNumber maxAccelerationDegPerSecSquared = new LoggedTunableNumber(armName + "/maxAccelerationDegPerSecSquared", ArmConstants.MAX_ACCELERATION_DEG_PER_SEC_SQUARED);

  private TrapezoidProfile armProfile;
  private TrapezoidProfile.Constraints armConstraints;

  private TrapezoidProfile.State armGoalStateDegrees = new TrapezoidProfile.State();
  private TrapezoidProfile.State armCurrentStateDegrees = new TrapezoidProfile.State();

  double goalDegrees;

  private ArmFeedforward armFFModel;

  /** Creates a new Arm. */
  public Arm(ArmIO arm) {
    this.arm = arm;
    switch (SimConstants.currentMode) {
      case REAL:
        kG.initDefault(TuningConstants.kP);
        kV.initDefault(TuningConstants.kV);
        kP.initDefault(TuningConstants.kP);
        kA.initDefault(TuningConstants.kA);
        kI.initDefault(TuningConstants.kI);
        kS.initDefault(TuningConstants.kS);
        kD.initDefault(TuningConstants.kD);
        break;
      case REPLAY:
        kG.initDefault(0);
        kV.initDefault(0);
        kP.initDefault(0);
        kA.initDefault(0);
        kI.initDefault(0);
        kS.initDefault(0);
        kD.initDefault(0);
        break;
      case SIM:
        kG.initDefault(0);
        kV.initDefault(0);
        kP.initDefault(0);
        kA.initDefault(0);
        kI.initDefault(0);
        kS.initDefault(0);
        kD.initDefault(0);
        break;
      default:
        kG.initDefault(TuningConstants.kP);
        kV.initDefault(TuningConstants.kV);
        kP.initDefault(TuningConstants.kP);
        kA.initDefault(TuningConstants.kA);
        kI.initDefault(TuningConstants.kI);
        kS.initDefault(TuningConstants.kS);
        kD.initDefault(TuningConstants.kD);
        break;
    }

    armConstraints = new TrapezoidProfile.Constraints(maxVelocityDegPerSec.get(),
        maxAccelerationDegPerSecSquared.get());
    armProfile = new TrapezoidProfile(armConstraints);

    armCurrentStateDegrees = armProfile.calculate(0, armCurrentStateDegrees, armGoalStateDegrees);

    arm.configurePID(kP.get(), kI.get(), kD.get());
    armFFModel = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get());
    updateTunableNumbers();
  }

  public void setBrakeMode(boolean bool) {
    arm.setBrakeMode(bool);
  }

  public double getArmPositionDegs() {
    return armInputs.positionDegs;
  }

  public boolean atGoal(double threshold) {
    return (Math.abs(armInputs.positionDegs - goalDegrees) <= threshold);
  }

  private double getArmError() {
    return armInputs.positionSetpointDegs - armInputs.positionDegs;
  }

  public void setPositionDegs(double positionDegs, double velocityDegsPerSec) {
    positionDegs = MathUtil.clamp(positionDegs, ArmConstants.LOWER_BOUND, ArmConstants.HIGHER_BOUND);
    arm.setPositionSetpointDegs(
        positionDegs,
        armFFModel.calculate(
            Units.degreesToRadians(positionDegs), Units.degreesToRadians(velocityDegsPerSec)));
  }

  public void armStop() {
    arm.stop();
  }

  public void setArmGoal(double goalDegrees) {
    this.goalDegrees = goalDegrees;
    armGoalStateDegrees = new TrapezoidProfile.State(goalDegrees, 0);
  }

  public void setArmCurrent(double currentDegrees) {
    armCurrentStateDegrees = new TrapezoidProfile.State(currentDegrees, 0);
  }

  public Command setArmTarget(double goalDegrees, double thresholdDegrees) {

    return new InstantCommand(() -> setArmGoal(goalDegrees), this)
        .until(() -> atGoal(thresholdDegrees));
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run

    arm.updateInputs(armInputs);

    armCurrentStateDegrees = armProfile.calculate(
        SubsystemConstants.LOOP_PERIOD_SECONDS, armCurrentStateDegrees, armGoalStateDegrees);

    setPositionDegs(armCurrentStateDegrees.position, armCurrentStateDegrees.velocity);

    Logger.processInputs("Arm", armInputs);
    Logger.recordOutput("Arm Error", getArmError());

    Logger.recordOutput("Arm Goal", goalDegrees);

    updateTunableNumbers();

  }

  private void updateTunableNumbers() {
    if (kP.hasChanged(hashCode()) || kI.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      arm.configurePID(kP.get(), kI.get(), 0);
    }

    if (kA.hasChanged(hashCode()) || kS.hasChanged(hashCode()) || kV.hasChanged(hashCode())
        || kG.hasChanged(hashCode())) {
      armFFModel = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get());
    }
  }
}
