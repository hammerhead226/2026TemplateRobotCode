package frc.robot.subsystems.headset;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Headset extends SubsystemBase {
    private final HeadsetIO headsetIO;
    private final HeadsetIOInputsAutoLogged inputs = new HeadsetIOInputsAutoLogged();

    public Headset(HeadsetIO headsetIO) {
        this.headsetIO = headsetIO;
    }

    @Override
    public void periodic() {
        headsetIO.commandPeriodic();
        headsetIO.updateInputs(inputs);
        Logger.processInputs("headset", inputs);
        // TODO: add alerts
    }

    public boolean isTrustworthy() {
        return (inputs.isConnected && inputs.latency < 10 && inputs.frameCount > 60 && inputs.tracking);
    }

    public void setPose(Pose3d pose3d) {
        headsetIO.setPose(pose3d);
    }
}
