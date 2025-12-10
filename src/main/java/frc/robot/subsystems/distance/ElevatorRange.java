package frc.robot.subsystems.distance;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ElevatorRange extends SubsystemBase { // Just an elevator.

    private final ElevatorRangeIO elevator;
    private final ElevatorRangeInputsAutoLogged eInputs = new ElevatorRangeInputsAutoLogged();
    private final String name = "ElevatorRange"; // Ideally should be in SubsystemsConstants and be used as a parameter.

    private final CANRange distance;

    public ElevatorRange(ElevatorRangeIO elevator, CANRange distance) {
        this.elevator = elevator;
        this.distance = distance;
    }

    public double getElevatorPosition() {
        if (distance.getDistance()
                < 6) { // Distance needed for encoder to be reliable (may be due to slack in the chain or some
            // mechanical obstacle)
            return distance.getDistance();
        }

        return eInputs.elevatorPositionInch;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Alliance", DriverStation.getAlliance().isPresent());

        elevator.updateInputs(eInputs);

        Logger.processInputs(name, eInputs);
    }
}
