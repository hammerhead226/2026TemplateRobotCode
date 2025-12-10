package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SimConstants;
import frc.robot.constants.SubsystemConstants;

public class Elevator extends SubsystemBase {

    private final ElevatorIO elevator;

    private final ElevatorIOInputsAutoLogged eInputs = new ElevatorIOInputsAutoLogged();

    private static final LoggedNetworkNumber kP = new LoggedNetworkNumber("Elevator/kP");
    private static final LoggedNetworkNumber kI = new LoggedNetworkNumber("Elevator/kI");

    private static final LoggedNetworkNumber kS = new LoggedNetworkNumber("Elevator/kS");
    private static final LoggedNetworkNumber kG = new LoggedNetworkNumber("Elevator/kG");
    private static final LoggedNetworkNumber kV = new LoggedNetworkNumber("Elevator/kV");
    private static final LoggedNetworkNumber kA = new LoggedNetworkNumber("Elevator/kA");

    private static final LoggedNetworkNumber barkG = new LoggedNetworkNumber("Bar/kG");

    // CHANGE THESE VALUES TO MATCH THE ELEVATOR
    private static final int maxVelocityExtender = 1;
    private static final int maxAccelerationExtender = 1;

    private TrapezoidProfile extenderProfile;
    private TrapezoidProfile.Constraints extenderConstraints =
            new TrapezoidProfile.Constraints(maxVelocityExtender, maxAccelerationExtender);
    private TrapezoidProfile.State extenderGoal = new TrapezoidProfile.State();
    private TrapezoidProfile.State extenderCurrent = new TrapezoidProfile.State();

    private double goal;
    private final ElevatorFeedforward elevatorFFModel;

    public Elevator(ElevatorIO elevator) {
        this.elevator = elevator;

        switch (SimConstants.currentMode) {
            case REAL:
                kS.setDefault(0);
                kG.setDefault(0);
                kV.setDefault(0);
                kA.setDefault(0);

                kP.setDefault(0);
                kI.setDefault(0);

                barkG.setDefault(0);
                break;
            case REPLAY:
                kS.setDefault(0);
                kG.setDefault(0);
                kV.setDefault(0);
                kA.setDefault(0);

                kP.setDefault(0);
                kI.setDefault(0);

                barkG.setDefault(0);
                break;
            case SIM:
                kS.setDefault(0);
                kG.setDefault(0);
                kV.setDefault(0);
                kA.setDefault(0);

                kP.setDefault(1);
                kI.setDefault(0);

                barkG.setDefault(0);
                break;
            default:
                kS.setDefault(0);
                kG.setDefault(0);
                kV.setDefault(0);
                kA.setDefault(0);

                kP.setDefault(0);
                kI.setDefault(0);

                barkG.setDefault(0);
                break;
        }

        // CHANGE THIS VALUE TO MATCH THE ELEVATOR
        setExtenderGoal(1);
        extenderProfile = new TrapezoidProfile(extenderConstraints);
        extenderCurrent = extenderProfile.calculate(0, extenderCurrent, extenderGoal);

        this.elevator.configurePID(kP.get(), 0, 0);
        elevatorFFModel = new ElevatorFeedforward(kS.get(), kG.get(), kV.get(), kA.get());
    }

    public boolean atGoal() {
        return (Math.abs(eInputs.elevatorPositionInch - goal)
                <= SubsystemConstants.ElevatorConstants.DEFAULT_THRESHOLD);
    }

    public boolean hasReachedGoal(double thresholdInches) {
        return Math.abs(eInputs.elevatorPositionInch - goal) <= thresholdInches;
    }

    public double getElevatorPosition() {
        return eInputs.elevatorPositionInch;
    }

    private double getElevatorError() {
        return eInputs.positionSetpointInch - eInputs.elevatorPositionInch;
    }

    public boolean elevatorAtSetpoint(double thersholdInches) {
        return (Math.abs(getElevatorError()) <= thersholdInches);
    }

    public void setExtenderGoal(double setpoint) {
        goal = setpoint;
        extenderGoal = new TrapezoidProfile.State(setpoint, 0);
    }

    public void setPositionExtend(double position, double velocity) {
        elevator.setPositionSetpoint(position, elevatorFFModel.calculate(velocity));
    }

    public void elevatorStop() {
        elevator.stop();
    }

    public double calculateAngle() {
        double angle = 0.0;
        return angle;
    }

    public void setConstraints(double maxVelocityMetersPerSec, double maxAccelerationMetersPerSecSquared) {
        extenderConstraints =
                new TrapezoidProfile.Constraints(maxVelocityMetersPerSec, maxAccelerationMetersPerSecSquared);
        extenderProfile = new TrapezoidProfile(extenderConstraints);
    }

    public boolean isExtended() {
        return extenderGoal.position == SubsystemConstants.ElevatorConstants.EXTEND_SETPOINT_INCH;
    }

    public Command setElevatorTarget(double goalInches, double thersholdInches) {

        return new InstantCommand(() -> setExtenderGoal(goalInches), this)
                .until(() -> elevatorAtSetpoint(thersholdInches));
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Alliance", DriverStation.getAlliance().isPresent());

        elevator.updateInputs(eInputs);

        extenderCurrent =
                extenderProfile.calculate(SubsystemConstants.LOOP_PERIOD_SECONDS, extenderCurrent, extenderGoal);

        setPositionExtend(extenderCurrent.position, extenderCurrent.velocity);

        Logger.processInputs("Elevator", eInputs);
    }
}
