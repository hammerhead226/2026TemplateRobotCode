// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arms;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SimConstants;
import frc.robot.constants.SubsystemConstants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

// TODO Add alerts for disconnects and motor temp, add subsystem vis
// TODO It looks like this class was based on the 2025 template code. I highly highly recommend basing it on the 2025
// actual code as there were a tremendous number of improvements

public class Arm extends SubsystemBase {
    private final ArmIO arm;
    private final ArmIOInputsAutoLogged pInputs = new ArmIOInputsAutoLogged();

    private static LoggedTunableNumber kP;
    private static LoggedTunableNumber kG;
    private static LoggedTunableNumber kV;
    private static LoggedTunableNumber kA;
    private static LoggedTunableNumber kS;
    private static LoggedTunableNumber kI;

    private static double maxVelocityDegPerSec;
    private static double maxAccelerationDegPerSecSquared;

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
                kG.initDefault(0.29);
                kV.initDefault(1);
                kP.initDefault(1.123);
                kA.initDefault(0);
                kS.initDefault(0);
                kI.initDefault(0);
                break;
            case REPLAY:
                kG.initDefault(0.29);
                kV.initDefault(1);
                kP.initDefault(1.123);
                kA.initDefault(0);
                kS.initDefault(0);
                kI.initDefault(0);
                break;
            case SIM:
                kG.initDefault(0.29);
                kV.initDefault(1);
                kP.initDefault(1.123);
                kA.initDefault(0);
                kS.initDefault(0);
                kI.initDefault(0);

                break;
            default:
                kG.initDefault(0.29);
                kV.initDefault(1);
                kP.initDefault(1.123);
                kA.initDefault(0);
                kS.initDefault(0);
                kI.initDefault(0);
                break;
        }

        // CHANGE PER ARM
        maxVelocityDegPerSec = 1;
        maxAccelerationDegPerSecSquared = 1;
        // maxAccelerationDegPerSecSquared = 180;

        armConstraints = new TrapezoidProfile.Constraints(maxVelocityDegPerSec, maxAccelerationDegPerSecSquared);
        armProfile = new TrapezoidProfile(armConstraints);

        // setArmGoal(90);
        // setArmCurrent(getArmPositionDegs());
        armCurrentStateDegrees = armProfile.calculate(0, armCurrentStateDegrees, armGoalStateDegrees);

        arm.configurePID(kP.get(), kI.get(), 0);
        armFFModel = new ArmFeedforward(kS.get(), kG.get(), kV.get(), 0);
    }

    public void setBrakeMode(boolean bool) {
        arm.setBrakeMode(bool);
    }

    public double getArmPositionDegs() {
        return pInputs.positionDegs;
    }

    public boolean atGoal(double threshold) {
        return (Math.abs(pInputs.positionDegs - goalDegrees) <= threshold);
    }

    public boolean hasReachedGoal(double thresholdDegrees) {
        return Math.abs(pInputs.positionDegs - goalDegrees) <= thresholdDegrees;
    }

    private double getArmError() {
        return pInputs.positionSetpointDegs - pInputs.positionDegs;
    }

    public boolean armAtSetpoint(double thresholdDegrees) {
        return (Math.abs(getArmError()) <= thresholdDegrees);
    }

    public void setConstraints(double maxVelocityDegreesPerSec, double maxAccelerationDegreesPerSecSquared) {
        armConstraints =
                new TrapezoidProfile.Constraints(maxVelocityDegreesPerSec, maxAccelerationDegreesPerSecSquared);
        armProfile = new TrapezoidProfile(armConstraints);
    }

    public void setPositionDegs(double positionDegs, double velocityDegsPerSec) {
        // positionDegs = MathUtil.clamp(positionDegs, 33, 120);
        arm.setPositionSetpointDegs(
                positionDegs,
                armFFModel.calculate(Units.degreesToRadians(positionDegs), Units.degreesToRadians(velocityDegsPerSec)));
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

        return new InstantCommand(() -> setArmGoal(goalDegrees), this).until(() -> atGoal(thresholdDegrees));
    }

    @Override
    public void periodic() {
        arm.updateInputs(pInputs);

        armCurrentStateDegrees = armProfile.calculate(
                SubsystemConstants.LOOP_PERIOD_SECONDS, armCurrentStateDegrees, armGoalStateDegrees);

        setPositionDegs(armCurrentStateDegrees.position, armCurrentStateDegrees.velocity);

        Logger.processInputs("Arm", pInputs);
        Logger.recordOutput("arm error", getArmError());

        Logger.recordOutput("arm goal", goalDegrees);
        // This method will be called once per scheduler run

        updateTunableNumbers();
    }

    private void updateTunableNumbers() {
        if (kP.hasChanged(hashCode()) || kI.hasChanged(hashCode())) {
            arm.configurePID(kP.get(), kI.get(), 0);
        }

        if (kS.hasChanged(hashCode())
                || kG.hasChanged(hashCode())
                || kV.hasChanged(hashCode())
                || kA.hasChanged(hashCode())) {
            armFFModel = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get());
        }
    }
}
