package org.firstinspires.ftc.teamcode.testingFiles;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "IntakePushIndex")
public class IntakePushIndex extends LinearOpMode {

    private DcMotor intake;
    private Servo push, index;
    private boolean onPush = true;
    private boolean onIndex = false;

    @Override
    public void runOpMode() {
        intake = hardwareMap.get(DcMotor.class, "Intake");
        push   = hardwareMap.get(Servo.class, "Push");
        index  = hardwareMap.get(Servo.class, "Index");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double pushPos  = 0.5;
        double indexPos = 0.5;
        double step = 0.1;
        double lastPushPos = pushPos;
        double lastIndexPos = indexPos;

        waitForStart();

        while (opModeIsActive()) {


            // Intake motor control - right bumper powers, left bumper turns off
            if (gamepad1.right_bumper) {
                intake.setPower(1.0);
                sleep(50);
                telemetry.addLine("Intake powered");
                telemetry.update();
            }

            else if (gamepad1.left_bumper) {
                intake.setPower(0.0);
                sleep(50);
                telemetry.addLine("Intake turned off");
                telemetry.update();
            }
// very cool comment for pushing
            if (gamepad1.a) {
                push.setPosition(pushPos);
                onPush = true;
                onIndex = false;
                sleep(50);
            }
            if (gamepad1.b) {
                index.setPosition(indexPos);
                onIndex = true;
                onPush = false;
                sleep(50);
            }
            if (gamepad1.dpad_up) {
                if (onPush) pushPos += step;
                if (onIndex) indexPos += step;
                sleep(50);
            }
            if (gamepad1.dpad_down) {
                if (onPush) pushPos -= step;
                if (onIndex) indexPos -= step;
                sleep(50);
            }
            if (pushPos != lastPushPos) {
                telemetry.addData("New push position: ", pushPos);
                telemetry.update();
                lastPushPos = pushPos;
                sleep(50);
            }
            if (indexPos != lastIndexPos) {
                telemetry.addData("New index position: ", indexPos);
                telemetry.update();
                lastIndexPos = indexPos;
                sleep(50);
            }

            push.setPosition(pushPos);
            index.setPosition(indexPos);

            telemetry.addData("Intake Power", intake.getPower());
            telemetry.addData("Push Pos", pushPos);
            telemetry.addData("Index Pos", indexPos);
            telemetry.update();
        }
    }


} 

