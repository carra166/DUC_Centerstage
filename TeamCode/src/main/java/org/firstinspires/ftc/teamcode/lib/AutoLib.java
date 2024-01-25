package org.firstinspires.ftc.teamcode.lib;

public class AutoLib {

    public static double calculateArmPower(double armAngle, double kCos, double kp, double target) {
        return kCos * Math.cos(Math.toRadians(armAngle)) + (kp * (target-armAngle));
    }

}
