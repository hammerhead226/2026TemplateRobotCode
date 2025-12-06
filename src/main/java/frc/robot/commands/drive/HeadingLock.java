// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.drive.holonomic.HolonomicDrive;
import frc.robot.commands.drive.holonomic.JoystickController;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HeadingLock extends Command {
    /** Creates a new HeadingLock. */
    private Drive drive;

    private final DoubleSupplier vxSupplier;
    private final DoubleSupplier vySupplier;
    private final DoubleSupplier omegaSupplier;

    private HolonomicDrive jStickDrive;
    private HolonomicDrive headingHoldDrive;

    Timer timer = new Timer();

    private Rotation2d targetAngle;

    public HeadingLock(
            Drive drive, DoubleSupplier vxSupplier, DoubleSupplier vySupplier, DoubleSupplier omegaSupplier) {
        addRequirements(drive);
        this.drive = drive;
        this.vxSupplier = vxSupplier;
        this.vySupplier = vySupplier;
        this.omegaSupplier = omegaSupplier;
    }

    @Override
    public void initialize() {
        targetAngle = drive.getPose().getRotation();

        jStickDrive = new HolonomicDrive(drive, (Supplier<ChassisSpeeds>) () -> JoystickController.getSpeeds(
                drive, vxSupplier.getAsDouble(), vySupplier.getAsDouble(), omegaSupplier.getAsDouble()));

        ProfiledPIDController rotationController =
                new ProfiledPIDController(1.0, 0, 0.4, new TrapezoidProfile.Constraints(8.0, 20.0));
        rotationController.reset(drive.getRotation().getRadians(), drive.getChassisSpeeds().omegaRadiansPerSecond);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        headingHoldDrive = new HolonomicDrive(drive, (Supplier<ChassisSpeeds>) () -> {
            ChassisSpeeds joystickControl = JoystickController.getSpeeds(
                    drive, vxSupplier.getAsDouble(), vySupplier.getAsDouble(), omegaSupplier.getAsDouble());

            rotationController.setGoal(targetAngle.getRadians());
            double headingLockControl =
                    rotationController.calculate(drive.getRotation().getRadians())
                            * drive.getMaxAngularSpeedRadPerSec();
            return new ChassisSpeeds(
                    joystickControl.vxMetersPerSecond, joystickControl.vyMetersPerSecond, headingLockControl);
        });
    }

    @Override
    public void execute() {
        // Once executed timer starts, if rotation is detected timer is reset and kept at 0
        // If no rotation is detected for 0.2s heading hold is activated
        double omega = omegaSupplier.getAsDouble();
        timer.start();
        if ((omega < -0.1 || 0.1 < omega)) {
            targetAngle = drive.getPose().getRotation();
            timer.reset();
            jStickDrive.execute();

        } else if (timer.get() > 1) {
            headingHoldDrive.execute();
        }
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
