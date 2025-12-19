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

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.LimelightHelpers;

/** IO implementation for real Limelight hardware. */
public class ObjectDetectionIOLimelight implements ObjectDetectionIO {
    // Second class for the limelight pointing DOWN so it doesn't constantly check for apriltags to
    // update orientation

    private final DoubleSubscriber latencySubscriber;
    private final DoubleSubscriber txSubscriber;
    private final DoubleSubscriber tySubscriber;
    private final DoubleSubscriber hbSubscriber;

    InterpolatingDoubleTreeMap distanceToObjectX = new InterpolatingDoubleTreeMap();
    InterpolatingDoubleTreeMap distanceToObjectY = new InterpolatingDoubleTreeMap();

    public ObjectDetectionIOLimelight(String name) {
        var table = NetworkTableInstance.getDefault().getTable(name);
        latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
        txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
        tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
        hbSubscriber = table.getDoubleTopic("hb").subscribe(0.0);

        distanceToObjectX.put(1.0, 57.80 + 4.0);
        distanceToObjectX.put(1.5, 49.00 + 2.5);
        distanceToObjectX.put(2.0, 42.75);
        distanceToObjectX.put(2.5, 36.70);
        distanceToObjectX.put(3.0, 34.00);
        distanceToObjectX.put(3.5, 31.20);
        distanceToObjectY.put(4.0, 29.50);
        distanceToObjectY.put(4.5, 27.50);
        distanceToObjectY.put(5.0, 26.00);
        distanceToObjectY.put(5.5, 24.85);
        distanceToObjectY.put(6.0, 23.60);
        distanceToObjectY.put(6.5, 22.80);
    }

    @Override
    public void updateInputs(VisionDetectionIOInputs inputs) {
        // Update connection status based on whether an update has been seen in the last 250ms
        inputs.connected = (RobotController.getFPGATime() - latencySubscriber.getLastChange()) < 250;
        inputs.heartBeat = hbSubscriber.get();
        // Update target observation
        inputs.latestTargetObservation = new TargetObservation(
                Rotation2d.fromDegrees(txSubscriber.get()), Rotation2d.fromDegrees(tySubscriber.get()));

        inputs.iTX = txSubscriber.get();
        inputs.iTY = tySubscriber.get();
        inputs.timestamp = txSubscriber.getAtomic().timestamp;
    }
    // TODO tx and ty don't both represent the objects distance from the camera necessarily, if the camera is mounted
    // pointing out from the robot tx will represent the angle from the camera and ty will represent the distance
    // TODO this should potentially be done at the objectdetection.java layer to follow the data flow logic of io
    // layers, and LimelightHelpers.getTX this is doing the same thing as the txSubscriber is
    public Pose2d getPose() {
        return new Pose2d(
                distanceToObjectX.get(Double.valueOf(LimelightHelpers.getTX(null))),
                distanceToObjectY.get(Double.valueOf(LimelightHelpers.getTY(null))),
                null
                // 18/Math.tan(Width/constant)
                );
    }
}
