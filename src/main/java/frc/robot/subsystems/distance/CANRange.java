package frc.robot.subsystems.distance;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class CANRange extends SubsystemBase {

    public CANRangeIO range;
    private final CANRangeIOInputsAutoLogged inputs = new CANRangeIOInputsAutoLogged();
    private final String name;

    public CANRange(CANRangeIO range, String name) {
        this.range = range;
        this.name = name;
    }

    public double getDistance() {
        return inputs.distanceInches;
    }

    public boolean isInRange(int min, int max) {
        return (getDistance() > min) && (getDistance() < max);
    }

    @Override
    public void periodic() {
        range.updateInputs(inputs);
        Logger.processInputs(name + "/CANrange", inputs);
    }
}
