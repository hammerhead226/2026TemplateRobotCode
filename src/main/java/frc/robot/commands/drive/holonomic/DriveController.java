package frc.robot.commands.drive.holonomic;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface DriveController {
    public ChassisSpeeds getSpeeds();

    public default void reset() {}
}
