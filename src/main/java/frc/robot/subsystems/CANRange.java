package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANRange extends SubsystemBase{

    public CANrange range;

    public CANRange(int canID, String canBusString) {
        range = new CANrange(0, canBusString);
    }

    public double getDistance() {
        return getDistance();
    }

    public boolean isInRange(int min, int max) {
        return (getDistance() > min) && (getDistance() < max); 
    }
}
