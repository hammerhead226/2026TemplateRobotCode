// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import com.therekrab.autopilot.Autopilot.APResult;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class autoPilot extends Command {
    /** Creates a new autoPilot. */
    private final Drive drive;

    private final Vision vision;

    private final APTarget target;

    private SwerveRequest.FieldCentricFacingAngle m_request = new SwerveRequest.FieldCentricFacingAngle()
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
            .withHeadingPID(4, 0, 0); /* tune this for your robot! */

    private static final APConstraints kConstraints =
            new APConstraints().withAcceleration(5.0).withJerk(2.0);

    private static final APProfile kProfile = new APProfile(kConstraints)
            .withErrorXY(Centimeters.of(2))
            .withErrorTheta(Degrees.of(0.5))
            .withBeelineRadius(Centimeters.of(8));

    private static final Autopilot kAutopilot = new Autopilot(kProfile);

    public autoPilot(Drive drive, Vision vision, APTarget target) {
        this.drive = drive;
        this.vision = vision;
        this.target = target;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drive, vision);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        ChassisSpeeds robotRelativeSpeeds = drive.getChassisSpeeds();
        Pose2d pose = drive.getPose();

        APResult out = autoPilot.kAutopilot.calculate(pose, robotRelativeSpeeds, target);

        final double P = 2;
        drive.runVelocity(new ChassisSpeeds(
                out.vx(),
                out.vy(),
                AngularVelocity.ofBaseUnits(
                        P
                                * (out.targetAngle().getRadians()
                                        - drive.getRotation().getRadians()),
                        RadiansPerSecond)));

        // m_request
        //     .withVelocityX(out.vx())
        //     .withVelocityY(out.vy())
        //     .withTargetDirection(out.targetAngle()));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
