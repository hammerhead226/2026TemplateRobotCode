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

// does not include swerve constants

package frc.robot.constants;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.Drive;

public final class SubsystemConstants {

  public static final String CANBUS = "CAN Bus 2";
  public static final double LOOP_PERIOD_SECONDS = 0.02;
  public static final boolean TUNING_MODE = true;

    public static class ElevatorConstants {
        public static final double CURRENT_LIMIT = 40.0;
        public static final boolean CURRENT_LIMIT_ENABLED = true;

        // the circumerfence of the sprocket that actually touches and drives the elevator chain
        // in 2025 22t 1/2* Hex Bore Hub 25 Chain Sprocket (217-2640) was 5.5
        public static final double SPROCKET_CIRCUMFERENCE_INCH = 5.5;

        public static final double RETRACT_SETPOINT_INCH = 0;
        public static final double EXTEND_SETPOINT_INCH = 0;
        public static final double DEFAULT_THRESHOLD = 1;

        public static final double ELEVATOR_GEAR_RATIO = 1;

        // Place Holder Values
        public static final double LOW_SETPOINT_INCH = 0;
        public static final double MID_SETPOINT_INCH = 0;
        public static final double HIGH_SETPOINT_INCH = 0;
        public static final double INTAKE_SETPOINT_INCH = 0;
        public static final double STOW_SETPOINT_INCH = 0;
    }

  public static class IndexerConstants {
    public static final String INDEXER_STRING = "defaultIndexer";
  }

  public static class FlywheelConstants {
    public static final double GEAR_RATIO = 1.5;
    public static final String FLYWHEEL_STRING = "defaultFlywheel";
  }

  public static final class ArmConstants {
    public static final String ARM_STRING = "defaultArm";
    public static final double CURRENT_LIMIT = 35.0;
    public static final boolean CURRENT_LIMIT_ENABLED = true;

    public static final double MAX_ACCELERATION_DEG_PER_SEC_SQUARED = 1;
    public static final double MAX_VELOCITY_DEG_PER_SEC = 1;

    public static final double ARM_INITIAL_POSITION = 0;
    public static final double ARM_DEFAULT_POSITION = 0;
   


    //TODO: After tuning on Elastic, update these values:
    //Note: These are real constants and apply to the physical robot.
    public static final class TuningConstants {

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kS = 0;
    public static final double kG = 0;
    public static final double kV = 0;
    public static final double kA = 0;
    public static final double kD = 0;

    }

        public static final double DEFAULT_THRESHOLD = 1;
        public static final double ARM_GEAR_RATIO = 1;

        public static final double LOWER_BOUND = 30;
        public static final double HIGHER_BOUND = 120;

        // Just fillers for state machine values
        public static final double LOW_SETPOINT_DEG = 0;
        public static final double MID_SETPOINT_DEG = 0;
        public static final double HIGH_SETPOINT_DEG = 0;
        public static final double INTAKE_ANGLE_DEGREES = 0;
    }

    public static final class PathConstants {
        public static final PathConstraints DEFAULT_PATH_CONSTRAINTS = new PathConstraints(
                5, // m/s
                5, // m/s^2
                Math.toRadians(180), // rad/s
                Math.toRadians(200) // rad/s^2
                );

        public static final Pose2d TAG_POSE2D_1 =
                new Pose2d(Units.feetToMeters(2), Units.feetToMeters(4), Rotation2d.kCCW_90deg);
        public static final Pose2d TAG_POSE2D_2 =
                new Pose2d(Units.feetToMeters(-2), Units.feetToMeters(-4), Rotation2d.kCCW_90deg);
        public static final Pose2d TAG_POSE2D_3 =
                new Pose2d(Units.feetToMeters(5), Units.feetToMeters(3), Rotation2d.kCCW_90deg);
        public static final Pose2d TAG_POSE2D_4 =
                new Pose2d(Units.feetToMeters(0), Units.feetToMeters(0), Rotation2d.kCCW_90deg);
        public static final Pose2d TAG_POSE2D_5 =
                new Pose2d(Units.feetToMeters(3), Units.feetToMeters(-2), Rotation2d.kCCW_90deg);

        // pathplanner.lib based commands
        public static final Translation2d ROUGH_TRANSLATION2D =
                new Translation2d(Units.feetToMeters(2), Units.feetToMeters(2));
        public static final PathConstraints ROUG_CONSTRAINTS = PathConstraints.unlimitedConstraints(12);
        public static final PathConstraints PRECISE_CONSTRAINTS = new PathConstraints(
                Drive.getMaxLinearSpeedMetersPerSec() * 0.5,
                3.0,
                Drive.getMaxAngularSpeedRadPerSec() * 0.5,
                Units.degreesToRadians(200),
                12.0);
    }

    public static enum LED_STATE {
        BLUE,
        RED,
        YELLOW,
        VIOLET,
        GREEN,
        GREY,
        PURPLE,
        PAPAYA_ORANGE,
        WILLIAMS_BLUE,
        HALF_FLASH_RED_HALF_FLASH_WHITE,
        FLASHING_WHITE,
        FLASHING_GREEN,
        FLASHING_RED,
        FLASHING_BLUE,
        FIRE,
        OFF
    }

    public static enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    public static enum ArmStates {
        STOW,
        INTAKE,
        LOW,
        MID,
        HIGH
    }

    public static enum ElevatorStates {
        STOW,
        INTAKE,
        LOW,
        MID,
        HIGH
    }

    public static enum FlywheelStates {
        OFF,
        LOWGOAL,
        HIGHGOAL
    }

    public static enum DriveStates {
        FULLSPEED,
        SLOW
    }

    public static enum SuperstructureState {
        IDLE,
        INTAKE,
        SCORELOW,
        SCOREMID,
        SCOREHIGH,
        STOW
    }
}
