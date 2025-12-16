package org.firstinspires.ftc.teamcode.subsystems;

public class GlobalVariable {

    //PID constants
    public double kP = 01.3;
    public double kI = 0.000001;
    public double kD = 0.0035;
    public static double integralSum = 0;
    public static double previousError = 0;
    public static double previousTime = 0;
    public boolean rumble = false;
    public static double close = 810;
    public static double far = 840;

    //nearBlue
    public static class nearBlue {
        public static double startX = -49;
        public static double startY = -49;
        public static double startH = Math.toRadians(230);
        public static double shootTopX = -12;
        public static double shootTopY = 0;
        public static double shootHalfX = 0;
        public static double shootHalfY = 0;
        public static double shootH = Math.toRadians(135);
        public static double startIntake1X;
        public static double startIntake1Y;
        public static double startIntake1H;
        public static double endIntake1X;
        public static double endIntake1Y;
        public static double endIntake1H;
        public static double startIntake2X;
        public static double startIntake2Y;
        public static double startIntake2H;
        public static double endIntake2X;
        public static double endIntake2Y;
        public static double endIntake2H;
    }
    public static class nearRed {
        public static double startX = -49;
        public static double startY = 49;
        public static double startH = Math.toRadians(130);
        public static double shootTopX = -12;
        public static double shootTopY = 0;
        public static double shootHalfX = 0;
        public static double shootHalfY = 0;
        public static double shootH = Math.toRadians(225);
        public static double startIntake1X;
        public static double startIntake1Y;
        public static double startIntake1H;
        public static double endIntake1X;
        public static double endIntake1Y;
        public static double endIntake1H;
        public static double startIntake2X;
        public static double startIntake2Y;
        public static double startIntake2H;
        public static double endIntake2X;
        public static double endIntake2Y;
        public static double endIntake2H;
    }
}
