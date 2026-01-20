package org.firstinspires.ftc.teamcode.subsystems;

public class GlobalVariable {

    //PIDF constants
    public static double kF = 0.00042;
    public static double kP = 0.00009;
    public static double targetVelocity = 600;
    public static double integralSum = 0;
    public static double previousError = 0;
    public static double previousTime = 0;
    public boolean rumble = false;
    public static double close = 650;
    public static double far = 800;
    public static double targetHoodClose = 0.7;
    public static double targetHoodFar = 0.5;
    public static double hoodMultClose = targetHoodClose/close;
    public static double hoodMultFar = targetHoodFar/far;
    public static double maxBearingErr = 4.5;
    public static double bearing;
    public static double lockSpeed = 0.1;
    public static double bearingErr = bearing - maxBearingErr;
    public static double velHoodPos = -100;
    public static double redTag = 0;
    public static double blueTag = 2;
    public static int mainTag;
    public static boolean LLstart = false;
    public static double shortHoodPos = 0.9;
    public static float seeyuhHSV[]  = {};
    public static int currentBallPos;
    public static int ball1Color; // 0 -> Green, 1 -> Purple
    public static int ball2Color;
    public static int ball3Color;
    //nearBlue
    public static class nearBlue {
        public static double startX = -49;
        public static double startY = -49;
        public static double startH = Math.toRadians(225);
        public static double shootLongX = -15;
        public static double shootShortX = -18.5;
        public static double shootShortY = -18.5;
        public static double shootH = Math.toRadians(225);
        public static double PreIntake1X = -9.5;
        public static double PreIntake1Y = -32;
        public static double PreIntakeH = Math.toRadians(270);
        public static double Intake1X = -9.5;
        public static double Intake1Y = -42;
        public static double IntakeH = Math.toRadians(270);
        public static double Intake2X = 12;
        public static double Intake2Y = -43;
        public static double Intake3X = 36;
        public static double Intake3Y = -23;
        public static double IntakeEnd3X = 36;
        public static double IntakeEnd3Y = -48;
        public static double shootHalfTargetSpeed = 500;
        public static double shootShortTargetSpeed = 490;

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
