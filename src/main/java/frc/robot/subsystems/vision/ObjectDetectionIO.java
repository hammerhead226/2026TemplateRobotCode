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
import org.littletonrobotics.junction.AutoLog;

public interface ObjectDetectionIO {
  @AutoLog
  public static class VisionDetectionIOInputs {
    public boolean connected = false;
    public TargetObservation latestTargetObservation =
        new TargetObservation(new Rotation2d(), new Rotation2d());
    public double heartBeat = 0;

    public double latency;
    public double tagSpan;
    //  public double avgTagDist;
    //  public double avgTagArea;

    public double iTX;
    public double iTY;
    public double timestamp;
    //  public double iTA;
    //  public double iHB;
    // public boolean iTV;

    // public double iTHOR;
    // public double iTVERT;

    //  public double iPIPELINELATENCY;
    // public double iCAPTURELATENCY;
    //
    // public double aTX;
    //  public double aTY;
    //  public double aTA;
    //  public double aHB;
    //  public boolean aTV;

    //  public double aTHOR;
    // public double aTVERT;

    // public double aPIPELINELATENCY;
    // public double aCAPTURELATENCY;

  }

  /** Represents the angle to a simple target, not used for pose estimation. */
  public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

  public default void updateInputs(VisionDetectionIOInputs inputs) {}

  public default Pose2d getPose() {return new Pose2d();}
}
