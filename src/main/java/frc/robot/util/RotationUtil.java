package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class RotationUtil {
    /**
     * returns the smallest angle  needed to go from start to end
     * result will be in (-180,180]
     */
    public static double deltaAngleDegrees(double start, double end) {
        double distance = (end - start)%360;
        if (distance < 0) distance += 360;
        return distance > 180 ? distance - 360 : distance;
    }

    public static double deltaAngleDegrees(Rotation2d start, Rotation2d end) {
        return deltaAngleDegrees(start.getDegrees(), end.getDegrees());
    }

    // unit tests
    // maybe one day we'll actually use a proper unit testing library
    // public static void main(String[] args) {
    //     System.out.println(deltaAngleDegrees(-30,20));
    //     System.out.println(deltaAngleDegrees(20,-30));
    //     System.out.println(deltaAngleDegrees(330,20));
    //     System.out.println(deltaAngleDegrees(20,330));
    //     System.out.println(deltaAngleDegrees(10,330));
    //     System.out.println(deltaAngleDegrees(10,-30));
    //     System.out.println(deltaAngleDegrees(30,-330));
    // }
}
