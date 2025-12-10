package frc.robot.util;

public class Conversions {

    /**
     * @param positionCounts CANCoder Position Counts
     * @param gearRatio Gear Ratio between CANCoder and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double CANcoderToDegrees(double positionCounts, double gearRatio) {
        return positionCounts * (360.0 / (gearRatio * 4096.0));
    }
    /**
     * @param radians
     * @return degrees
     */
    public static double radiansToDegrees(double radians) {
        return radians * 180.0 / Math.PI;
    }
    /**
     * @param degrees
     * @return radians
     */
    public static double degreesToRadians(double degrees) {
        return degrees * Math.PI / 180.0;
    }
    /**
     * @param degrees Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between CANCoder and Mechanism
     * @return CANCoder Position Counts
     */
    public static double degreesToCANcoder(double degrees, double gearRatio) {
        return degrees / (360.0 / (gearRatio * 4096.0));
    }

    /**
     * @param counts Falcon Position Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double falconToDegrees(double positionCounts, double gearRatio) {
        // return positionCounts * (360.0 / (gearRatio * 2048.0));
        return positionCounts * (360.0 / (gearRatio));
    }

    /**
     * @param degrees Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Falcon Position Counts
     */
    public static double degreesToFalcon(double degrees, double gearRatio) {
        // return degrees / (360.0 / (gearRatio * 2048.0));
        return degrees / (360.0 / (gearRatio));
    }

    /**
     * @param velocityCounts Falcon Velocity Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return RPM of Mechanism
     */
    public static double falconToRPM(double velocityCounts, double gearRatio) {
        double motorRPM = velocityCounts * (600.0 / 2048.0);
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;
    }

    /**
     * @param RPM RPM of mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return RPM of Mechanism
     */
    public static double RPMToFalcon(double RPM, double gearRatio) {
        double motorRPM = RPM * gearRatio;
        double sensorCounts = motorRPM * (2048.0 / 600.0);
        return sensorCounts;
    }

    /**
     * @param velocitycounts Falcon Velocity Counts
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon MPS)
     * @return Falcon Velocity Counts
     */
    public static double falconToMPS(double velocitycounts, double circumference, double gearRatio) {
        double wheelRPM = falconToRPM(velocitycounts, gearRatio);
        double wheelMPS = (wheelRPM * circumference) / 60;
        return wheelMPS;
    }

    /**
     * @param velocity Velocity MPS
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon MPS)
     * @return Falcon Velocity Counts
     */
    public static double MPSToFalcon(double velocity, double circumference, double gearRatio) {
        double wheelRPM = ((velocity * 60) / circumference);
        double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
        return wheelVelocity;
    }

    /**
     * @param positionCounts Falcon Position Counts
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Wheel
     * @return Meters
     */
    public static double falconToMeters(double positionCounts, double circumference, double gearRatio) {
        return positionCounts * (circumference / (gearRatio * 2048.0));
    }

    /**
     * @param meters Meters
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Wheel
     * @return Falcon Position Counts
     */
    public static double metersToFalcon(double meters, double circumference, double gearRatio) {
        return meters / (circumference / (gearRatio * 2048.0));
    }
    /**
     * @param meters Meters
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Wheel
     * @return Motor Rotations
     */
    public static double MetersToMotorRot(double meters, double circumference, double gearRatio) {
        return meters / (circumference / (gearRatio));
    }
    /**
     * @param inches Inches
     * @param circumference Circumference of Wheel (Inches)
     * @param gearRatio Gear Ratio between Falcon and Wheel
     * @return Motor Rotations
     */
    public static double inchesToMotorRot(double inches, double circumference, double gearRatio) {
        return inches / (circumference / (gearRatio));
    }

    /**
     * @param motorPositionRot Motor Position Inches
     * @param circumference Circumference of Wheel (Inches)
     * @param gearRatio Gear Ratio between Motor and Mechanism
     * @return Position of Motor in Inches
     */
    public static double motorRotToInches(double motorPositionRot, double circumference, double gearRatio) {
        return motorPositionRot * (circumference / (gearRatio));
    }
    /**
     * @param velocityInches Mechanism Velocity Inches
     * @param circumference Circumference of Wheel (Inches)
     * @param gearRatio Gear Ratio between Motor and Mechanism
     * @return RPM of Motor
     */
    public static double IPSToMotorRPM(double velocityIPS, double circumference, double gearRatio) {
        return velocityIPS / (circumference / (gearRatio)) * 60.0;
    }
    /**
     * @param velocityRPM Motor Velocity RPM
     * @param circumference Circumference of Wheel (Inches)
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Mechanism Velocity in Inches Per Second
     */
    public static double motorRPMToIPS(double velocityRPM, double circumference, double gearRatio) {
        return velocityRPM * (circumference / (gearRatio)) / 60.0;
    }
    /**
     * @param accelerationInchPerSecSec Mechanism Acceleration Inch Per Sec^2
     * @param circumference Circumference of Wheel (Inches)
     * @param gearRatio Gear Ratio between Motor and Mechanism
     * @return Motor Accelearation RPM Per Sec
     */
    public static double IPSSToMotorRPMS(double accelerationInchPerSecSec, double circumference, double gearRatio) {
        return accelerationInchPerSecSec / (circumference / (gearRatio)) * 60.0;
    }
}
