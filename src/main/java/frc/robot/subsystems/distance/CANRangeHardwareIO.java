package frc.robot.subsystems.distance;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;

public class CANRangeHardwareIO implements CANRangeIO {
    private final CANrange range;
    private StatusSignal<Distance> distanceMeters;

    public CANRangeHardwareIO(int id, String canBusString) {
        range = new CANrange(id, canBusString);
        distanceMeters = range.getDistance();

        CANrangeConfiguration config = new CANrangeConfiguration();
        config.FovParams.FOVRangeX = 10;
        config.FovParams.FOVRangeY = 10;
        range.getConfigurator().apply(config);

        BaseStatusSignal.setUpdateFrequencyForAll(100, distanceMeters);
    }

    @Override
    public void updateInputs(CANRangeIOInputs inputs) {
        BaseStatusSignal.refreshAll(distanceMeters);
        inputs.connected = range.isConnected();
        inputs.distanceInches = Units.metersToInches(distanceMeters.getValueAsDouble());
        inputs.distanceStd = Units.metersToInches(range.getDistanceStdDev().getValueAsDouble());
    }
}
