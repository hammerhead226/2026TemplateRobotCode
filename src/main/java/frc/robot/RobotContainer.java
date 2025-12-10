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

package frc.robot;

import static frc.robot.constants.VisionConstants.camera0Name;
import static frc.robot.constants.VisionConstants.camera1Name;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.drive.holonomic.HolonomicDrive;
import frc.robot.commands.drive.holonomic.JoystickController;
import frc.robot.commands.drive.holonomic.PIDPoseController;
import frc.robot.constants.SimConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.arms.Arm;
import frc.robot.subsystems.arms.ArmIO;
import frc.robot.subsystems.arms.ArmIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.headset.Headset;
import frc.robot.subsystems.headset.HeadsetIOQuestNav;
import frc.robot.subsystems.vision.ObjectDetection;
import frc.robot.subsystems.vision.ObjectDetectionIO;
import frc.robot.subsystems.vision.ObjectDetectionIOLimelight;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    public static Drive drive;
    private final Flywheel flywheel;
    private final Arm arm;
    private final Vision vision;
    private final Headset headset;
    private final ObjectDetection objectDetection;

    // Controller
    private final CommandXboxController controller = new CommandXboxController(0);

    Pose2d targetPoseTest = new Pose2d(
            Units.inchesToMeters(31.526), Units.inchesToMeters(297.176), Rotation2d.fromDegrees(90 - 144.011));

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        switch (SimConstants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drive = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
                        new ModuleIOTalonFX(TunerConstants.FrontRight),
                        new ModuleIOTalonFX(TunerConstants.BackLeft),
                        new ModuleIOTalonFX(TunerConstants.BackRight));

                arm = new Arm(new ArmIOSim());
                flywheel = new Flywheel(new FlywheelIOSim());
                vision = new Vision(
                        drive::addVisionMeasurement,
                        new VisionIOLimelight(camera0Name, drive::getRotation),
                        new VisionIOLimelight(camera1Name, drive::getRotation));
                objectDetection = new ObjectDetection(
                        drive::addObjectMeasurement,
                        new ObjectDetectionIOLimelight(VisionConstants.cameraObjectDetect));

                headset = new Headset(new HeadsetIOQuestNav());

                // drive = new Drive(
                // new GyroIOPigeon2(true),
                // new ModuleIOTalonFX(0),
                // new ModuleIOTalonFX(1),
                // new ModuleIOTalonFX(2),
                // new ModuleIOTalonFX(3));
                // flywheel = new Flywheel(new FlywheelIOTalonFX());
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                arm = new Arm(new ArmIOSim());
                drive = new Drive(
                        new GyroIO() {},
                        new ModuleIOSim(TunerConstants.FrontLeft),
                        new ModuleIOSim(TunerConstants.FrontRight),
                        new ModuleIOSim(TunerConstants.BackLeft),
                        new ModuleIOSim(TunerConstants.BackRight));
                vision = new Vision(
                        drive::addVisionMeasurement,
                        new VisionIOPhotonVisionSim(camera0Name, VisionConstants.robotToCamera0, drive::getPose),
                        new VisionIOPhotonVisionSim(camera1Name, VisionConstants.robotToCamera1, drive::getPose));
                objectDetection = new ObjectDetection(drive::addObjectMeasurement, new ObjectDetectionIO() {});
                flywheel = new Flywheel(new FlywheelIOSim());
                headset = null;
                break;

            default:
                // Replayed robot, disable IO implementations
                arm = new Arm(new ArmIO() {});
                drive = new Drive(
                        new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
                flywheel = new Flywheel(new FlywheelIO() {});
                vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
                objectDetection = new ObjectDetection(drive::addObjectMeasurement, new ObjectDetectionIO() {});
                headset = null;
                break;
        }

        // Set up auto routines
        NamedCommands.registerCommand(
                "Run Flywheel",
                Commands.startEnd(() -> flywheel.runVelocity(500), flywheel::stop, flywheel)
                        .withTimeout(5.0));
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Flywheel SysId (Quasistatic Forward)", flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Flywheel SysId (Quasistatic Reverse)", flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Flywheel SysId (Dynamic Forward)", flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Flywheel SysId (Dynamic Reverse)", flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(new HolonomicDrive(drive, (Supplier<ChassisSpeeds>) () -> (JoystickController.getSpeeds(
                drive, controller.getLeftX(), controller.getLeftY(), controller.getRightX()))));

        PIDPoseController rotationController = new PIDPoseController(drive, drive::getPose, () -> Pose2d.kZero);

        // Lock to 0° when A button is held
        controller.a().whileTrue(new HolonomicDrive(drive, (Supplier<ChassisSpeeds>) () -> {
            ChassisSpeeds joystickControl = JoystickController.getSpeeds(
                    drive, controller.getLeftX(), controller.getLeftY(), controller.getRightX());
            ChassisSpeeds rotationControl = rotationController.getSpeeds();
            return new ChassisSpeeds(
                    joystickControl.vxMetersPerSecond,
                    joystickControl.vyMetersPerSecond,
                    rotationControl.omegaRadiansPerSecond);
        }));

        // Reset drive pose to estimate on start pressed
        controller.start().onTrue(new InstantCommand(() -> drive.setPose(Pose2d.kZero)));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public Drive getDrive() {
        return drive;
    }

    public Flywheel getFlywheel() {
        return flywheel;
    }

    public Arm getArm() {
        return arm;
    }

    public Vision getVision() {
        return vision;
    }

    public Headset getHeadset() {
        return headset;
    }

    public ObjectDetection getObjectDetection() {
        return objectDetection;
    }
}
