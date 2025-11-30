package org.firstinspires.ftc.teamcode.testingFiles;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "IntakePushIndex")
public class IntakePushIndex extends LinearOpMode {

    private DcMotor intake, flywheel;
    private Servo push, index;
    private boolean onPush = true;
    private boolean onIndex = false;

    @Override
    public void runOpMode() {
        intake = hardwareMap.get(DcMotor.class, "Intake");
        flywheel = hardwareMap.get(DcMotor.class, "Flywheel");
        push   = hardwareMap.get(Servo.class, "Push");
        index  = hardwareMap.get(Servo.class, "Index");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel.setDirection(DcMotor.Direction.REVERSE);


        double pushPos  = 0.5;
        double indexPos = 0.5; //0.5 -> 0.9
        double step = 0.1;
        double lastPushPos = pushPos;
        double lastIndexPos = indexPos;
        long slep = 150;

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addLine("Flywheel - Right/Left Bumper");
            telemetry.addLine("Push - a");
            telemetry.addLine("Index - b");
            telemetry.addLine("Increase/Decrease Servo Positions - Dpad up/down");


            // Intake motor control - right bumper powers, left bumper turns off
            if (gamepad1.right_bumper) {
                flywheel.setPower(1.0);
                sleep(slep);
                telemetry.addLine("Intake powered");
                telemetry.update();
            }

            else if (gamepad1.left_bumper) {
                flywheel.setPower(0.0);
                sleep(slep);
                telemetry.addLine("Intake turned off");
                telemetry.update();
            }
// very cool comment for pushing
            if (gamepad1.x) {
                pushPos = 1;
            }
            if (gamepad1.y) {
                pushPos = 0.5;
            }

            if (gamepad1.a) {
                onPush = true;
                onIndex = false;
                sleep(slep);
            }
            if (gamepad1.b) {
                onIndex = true;
                onPush = false;
                sleep(slep);
            }
            if (gamepad1.dpad_up) {
                if (onPush) {
                    pushPos += step;
                    if(pushPos >= 0.9) pushPos =0.9;
                }
                if (onIndex) indexPos += step;
                sleep(slep);
            }
            if (gamepad1.dpad_down) {
                if (onPush) pushPos -= step;
                if (onIndex) indexPos -= step;
                if(pushPos <= 0.5) pushPos =0.5;
                sleep(slep);
            }
            if (pushPos != lastPushPos) {
                telemetry.addData("New push position: ", pushPos);
                telemetry.update();
                lastPushPos = pushPos;
                sleep(slep);
            }
            if (indexPos != lastIndexPos) {
                telemetry.addData("New index position: ", indexPos);
                telemetry.update();
                lastIndexPos = indexPos;
                sleep(slep);
            }

            telemetry.addData("Intake Power", intake.getPower());
            telemetry.addData("Push Pos", pushPos);
            telemetry.addData("Index Pos", indexPos);
            telemetry.update();

            if (gamepad1.a) {
                push.setPosition(pushPos);
            }

            if (gamepad1.b) {
                index.setPosition(indexPos);
            }
        }
    }


} 

