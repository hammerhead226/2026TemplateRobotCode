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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** IO implementation for real PhotonVision hardware. */
public class ObjectDetectionIOPhotonVision implements ObjectDetectionIO {
    protected final PhotonCamera camera;
    protected final Transform3d robotToCamera;

    /**
     * Creates a new VisionIOPhotonVision.
     *
     * @param name The configured name of the camera.
     * @param rotationSupplier The 3D position of the camera relative to the robot.
     */
    public ObjectDetectionIOPhotonVision(String name, Transform3d robotToCamera) {
        camera = new PhotonCamera(name);
        this.robotToCamera = robotToCamera;
    }

    @Override
    public void updateInputs(VisionDetectionIOInputs inputs) {
        inputs.connected = camera.isConnected();

        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        Optional<PhotonPipelineResult> maybeLatestResult = results.stream().max((result1, result2) ->
                (int) Math.signum(result1.getTimestampSeconds() - result2.getTimestampSeconds()));
        if (maybeLatestResult.isEmpty()) return;
        PhotonPipelineResult latestResult = maybeLatestResult.get();

        List<PhotonTrackedTarget> targets = latestResult.getTargets();
        if (targets.size() == 0) return;
        PhotonTrackedTarget firstTarget = targets.get(0);

        inputs.latestTarget = new TargetObservation(
                Rotation2d.fromDegrees(firstTarget.getYaw()),
                Rotation2d.fromDegrees(firstTarget.getPitch()),
                latestResult.getTimestampSeconds());
    }
}
