package frc.robot.commands.drive.holonomic;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;
import frc.robot.commands.drive.holonomic.PIDPoseController;

public class HolonomicDrive extends Command {
    private Drive drive;
    private Supplier<ChassisSpeeds> speedsSupplier;
    private PIDPoseController pidPoseController;

    public HolonomicDrive(Drive drive) {
        addRequirements(drive);
        this.drive = drive;
        this.speedsSupplier = () -> new ChassisSpeeds();
    }

    public HolonomicDrive(Drive drive, Supplier<ChassisSpeeds> speedsSupplier) {
        this(drive);
        this.speedsSupplier = speedsSupplier;
    }
    // public HolonomicDrive(PIDPoseController pidPoseController, Supplier<ChassisSpeeds> speedsSupplier) {
    //     this.pidPoseController = pidPoseController;
    //     this.speedsSupplier = speedsSupplier;
    // }

    public HolonomicDrive(Drive drive, Supplier<ChassisSpeeds> speedsSupplier, Runnable... resetRunnables) {
        this(drive, speedsSupplier);
        for (Runnable resetRunnable : resetRunnables) {
            resetRunnable.run();
        }
    }

    @Override
    public void execute() {
        // Convert to field relative speeds & send command
        drive.runVelocity(speedsSupplier.get());
    }
}
