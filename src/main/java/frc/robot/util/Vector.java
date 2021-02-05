package frc.robot.util;

public class Vector {
    private double x;
    private double y;

    public Vector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getMagnitude() {
        return Math.sqrt((x * x) + (y * y));
    }

    public double getAngle() {
        return Math.toDegrees(Math.atan2(y, x));
    }
}
