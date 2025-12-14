package frc.robot.commands.drive.holonomic;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

public class HolonomicDrive extends Command {
    private Drive drive;
    private Supplier<ChassisSpeeds> speedsSupplier;
    private Runnable[] resetRunnables;

    public HolonomicDrive(Drive drive) {
        addRequirements(drive);
        this.drive = drive;
        this.speedsSupplier = () -> new ChassisSpeeds();
    }

    public HolonomicDrive(Drive drive, Supplier<ChassisSpeeds> speedsSupplier, Runnable... resetRunnables) {
        addRequirements(drive);
        this.drive = drive;
        this.speedsSupplier = speedsSupplier;
        this.resetRunnables = resetRunnables;
    }

    public HolonomicDrive(Drive drive, DriveController driveController) {
        this(drive, (Supplier<ChassisSpeeds>) driveController::getSpeeds, driveController::reset);
    }

    @Override
    public void initialize() {
        if (resetRunnables != null) {
            for (Runnable resetRunnable : resetRunnables) {
                resetRunnable.run();
            }
        }
    }

    @Override
    public void execute() {
        drive.runVelocity(speedsSupplier.get());
    }
}
