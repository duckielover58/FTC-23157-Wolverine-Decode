package org.firstinspires.ftc.teamcode.subsystems;

public class GlobalVariable {

    //PIDF constants
    public static double kF = 0.00042;
    public static double kP = 0.00009;
    public static double targetVelocity = 740;
    public static double integralSum = 0;
    public static double previousError = 0;
    public static double previousTime = 0;
    public boolean rumble = false;
    public static double close = 700;
    public static double far = 840;

    //nearBlue
    public static class nearBlue {
        public static double startX = -49;
        public static double startY = -49;
        public static double startH = Math.toRadians(230);
        public static double shootLongX = -15;
        public static double shootShortX = -18.5;
        public static double shootShortY = -18.5;
        public static double shootH = Math.toRadians(225);
        public static double Intake1X = -12;
        public static double Intake1Y = -43;
        public static double IntakeH = Math.toRadians(270);
        public static double Intake2X = 12;
        public static double Intake2Y = -43;
        public static double Intake3X = 36;
        public static double Intake3Y = -23;
        public static double IntakeEnd3X = 36;
        public static double IntakeEnd3Y = -48;

    }
    public static class nearRed {
        public static double startX = -49;
        public static double startY = 49;
        public static double startH = Math.toRadians(130);
        public static double shootTopX = -15;
        public static double shootTopY = 0;
        public static double shootHalfX = 0;
        public static double shootHalfY = 0;
        public static double shootH = Math.toRadians(225);
        public static double startIntake1X = -12;
        public static double startIntake1Y = -43;
        public static double IntakeH = Math.toRadians(270);
        public static double endIntake1X = -12;
        public static double endIntake1Y = -43;
        public static double startIntake2X = 12;
        public static double startIntake2Y = -43;
        public static double startIntake2H;
        public static double endIntake2X;
        public static double endIntake2Y;
    }
}
