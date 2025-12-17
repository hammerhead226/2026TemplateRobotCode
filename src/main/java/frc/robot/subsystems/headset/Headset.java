package frc.robot.subsystems.headset;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;
import gg.questnav.questnav.PoseFrame;
import org.littletonrobotics.junction.Logger;

public class Headset extends SubsystemBase {
    private final VisionConsumer consumer;
    private final HeadsetIO headsetIO;
    private final HeadsetIOInputsAutoLogged inputs = new HeadsetIOInputsAutoLogged();

    public Headset(VisionConsumer consumer, HeadsetIO headsetIO) {
        this.consumer = consumer;
        this.headsetIO = headsetIO;
    }

    @Override
    public void periodic() {
        headsetIO.commandPeriodic();
        headsetIO.updateInputs(inputs);
        for (PoseFrame poseFrame : inputs.poseFrames) {
            consumer.accept(
                    poseFrame
                            .questPose3d()
                            .transformBy(inputs.robotToHeadset.inverse())
                            .toPose2d(),
                    poseFrame.dataTimestamp(),
                    VecBuilder.fill(
                            VisionConstants.linearStdDevBaseline,
                            VisionConstants.linearStdDevBaseline,
                            VisionConstants.angularStdDevBaseline));
        }
        Logger.processInputs("headset", inputs);
        // TODO: add alerts
    }

    public void setPose(Pose3d pose3d) {
        headsetIO.setPose(pose3d);
    }

    @FunctionalInterface
    public static interface VisionConsumer {
        public void accept(
                Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs);
    }
}
