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

public final class SubsystemConstants {

    public static final String CANBUS = "CAN Bus 2";
    public static final double LOOP_PERIOD_SECONDS = 0.02;

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
    }

    public static final class ArmConstants {
        public static final double CURRENT_LIMIT = 35.0;
        public static final boolean CURRENT_LIMIT_ENABLED = true;

        public static final double DEFAULT_THRESHOLD = 1;
        public static final double ARM_GEAR_RATIO = 1;

        public static final double ARM_ZERO_ANGLE = 0;
        public static final double STOW_SETPOINT_DEG = 0;
    }

    public static final class PathConstants {
        public static final PathConstraints DEFAULT_PATH_CONSTRAINTS = new PathConstraints(
                5, // m/s
                5, // m/s^2
                Math.toRadians(180), // rad/s
                Math.toRadians(200) // rad/s^2
                );
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
}
