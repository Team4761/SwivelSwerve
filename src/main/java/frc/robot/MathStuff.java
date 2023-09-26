package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;

public class MathStuff {
    // % except negative numbers return positive ex. -13%63 = 51 instead of -13
    public static double modulus(double a, double b) {
        double c = a%b;
        if(c<0) {
            c+=b;
        }
        return c;
    }
    //angle subtraction but will result in -180 to 180 degrees
    public static Rotation2d subtract(Rotation2d a, Rotation2d b) {
        return new Rotation2d(
        modulus(a.getRadians() - b.getRadians() + Math.PI , Math.PI*2) - Math.PI);
        //(a%360 - b%360 + 180 ) % 360 - 180;
        // initial mod 360 not needed 
    }
}
