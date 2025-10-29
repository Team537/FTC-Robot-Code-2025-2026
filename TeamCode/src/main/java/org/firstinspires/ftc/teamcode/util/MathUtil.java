package org.firstinspires.ftc.teamcode.util;

public final class MathUtil {
    private MathUtil() {} // prevent instantiation

    /** Clamp a value between min and max */
    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /** Compare two doubles with a tolerance (epsilon) */
    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return Math.abs(a - b) <= epsilon;
    }

}
