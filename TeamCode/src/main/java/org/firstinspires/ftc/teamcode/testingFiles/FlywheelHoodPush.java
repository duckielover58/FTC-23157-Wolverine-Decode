package org.firstinspires.ftc.teamcode.testingFiles;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name = "FlywheelHoodSwivel")
public class FlywheelHoodPush extends LinearOpMode {

    private DcMotorEx flywheel;
    private Servo hood;
    private Servo push;
    private boolean onPush = true;
    private boolean onHood = false;
    private boolean onFlywheel = false;

    @Override
    public void runOpMode() {
        flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
        hood   = hardwareMap.get(Servo.class, "Hood");
        push  = hardwareMap.get(Servo.class, "Push");

        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double flywheelPower = 10/13.29;
        double hoodPos  = 0.5;
        double pushMax = 0.35;
        double pushMin = 0;
        double pushPos = 0;
        double step = 0.1;
        double lastHoodPos = hoodPos;
        double lastPushPos = pushPos;
        long slep = 150;

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addLine("Flywheel - Right/Left Bumper");
            telemetry.addLine("Hood - a");
            telemetry.addLine("Push - b");
            telemetry.addLine("Increase/Decrease Servo Positions - Dpad up/down");
            telemetry.addLine("Increase/Decrease Flywheel Position - Right Bumper is Up/Left Bumper is down");


// very cool comment for pushing
            if (gamepad1.a) {
                onPush = true;
                onHood = false;
                onFlywheel = false;
                sleep(slep);
            }
            if (gamepad1.b) {
                onHood = true;
                onPush = false;
                onFlywheel = false;
                sleep(slep);
            }
            if (gamepad1.x) {
                onHood = false;
                onPush = false;
                onFlywheel = true;
                sleep(slep);
            }

            if (gamepad1.dpad_up) {
                if (onPush) {
                    pushPos += step;
                    if(pushPos >= pushMax) pushPos = pushMax;
                }
                if (onHood) hoodPos += step;
                sleep(slep);
            }

            if (gamepad1.dpad_down) {
                if (onPush) pushPos -= step;
                if (onHood) hoodPos -= step;
                sleep(slep);
            }
            if (gamepad1.right_bumper) {
                flywheelPower += step;
                flywheelPower = Math.round((int) (flywheelPower));
                sleep(slep);
            }

            else if (gamepad1.left_bumper) {
                flywheelPower -= step;
                flywheelPower = Math.round((int) (flywheelPower));
                sleep(slep);
                telemetry.addLine("New Flywheel position:" + flywheelPower);
            }

            if (hoodPos != lastHoodPos) {
                telemetry.addData("New hood position: ", hoodPos);
                lastHoodPos = hoodPos;
                sleep(slep);
            }

            if (pushPos != lastPushPos) {
                telemetry.addData("New push position: ", pushPos);
                lastPushPos = pushPos;
                sleep(slep);
            }

            telemetry.addData("Flywheel Power", flywheel.getPower());
            telemetry.addData("Hood Pos", hoodPos);
            telemetry.addData("Push Pos", pushPos);
            telemetry.update();

            if (gamepad1.a) {
                hood.setPosition(hoodPos);
            }

            if (gamepad1.b) {
                push.setPosition(pushPos);
            }

            if (gamepad1.x) {
                flywheel.setPower(flywheelPower);
                telemetry.addLine("New Flywheel position:" + flywheelPower);
                telemetry.addData("Flywheel Velocity:", flywheel.getVelocity());
                telemetry.update();
            }
        }
    }


}

