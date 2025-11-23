package org.firstinspires.ftc.teamcode.testingFiles;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "FlywheelHoodSwivel")
public class FlywheelHoodSwivel extends LinearOpMode {

    private DcMotor flywheel;
    private Servo hood, swivel;
    private boolean onHood = true;
    private boolean onSwivel = false;

    @Override
    public void runOpMode() {
        flywheel = hardwareMap.get(DcMotor.class, "Flywheel");
        hood   = hardwareMap.get(Servo.class, "Hood");
        swivel  = hardwareMap.get(Servo.class, "Swivel");

        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double hoodPos  = 0.5;
        double swivelPos = 0.5;
        double step = 0.1;
        double lastHoodPos = hoodPos;
        double lastSwivelPos = swivelPos;
        long slep = 150;

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addLine("Flywheel - Right/Left Bumper");
            telemetry.addLine("Hood - a");
            telemetry.addLine("Swivel - b");
            telemetry.addLine("Increase/Decrease Servo Positions - Dpad up/down");


            if (gamepad1.right_bumper) {
                flywheel.setPower(1.0);
                sleep(slep);
                telemetry.addLine("Flywheel powered");
                telemetry.update();
            }

            else if (gamepad1.left_bumper) {
                flywheel.setPower(0.0);
                sleep(slep);
                telemetry.addLine("Flywheel turned off");
                telemetry.update();
            }
// very cool comment for pushing
            if (gamepad1.a) {

                onHood = true;
                onSwivel = false;
                sleep(slep);
            }

            if (gamepad1.b) {
                onSwivel = true;
                onHood = false;
                sleep(slep);
            }

            if (gamepad1.dpad_up) {
                if (onHood) hoodPos += step;
                if (onSwivel) swivelPos += step;
                sleep(slep);
            }

            if (gamepad1.dpad_down) {
                if (onHood) hoodPos -= step;
                if (onSwivel) swivelPos -= step;
                sleep(slep);
            }

            if (hoodPos != lastHoodPos) {
                telemetry.addData("New hood position: ", hoodPos);
                telemetry.update();
                lastHoodPos = hoodPos;
                sleep(slep);
            }

            if (swivelPos != lastSwivelPos) {
                telemetry.addData("New swivel position: ", swivelPos);
                telemetry.update();
                lastSwivelPos = swivelPos;
                sleep(slep);
            }

            telemetry.addData("Flywheel Power", flywheel.getPower());
            telemetry.addData("Hood Pos", hoodPos);
            telemetry.addData("Swivel Pos", swivelPos);
            telemetry.update();

            if (gamepad1.a) {
                hood.setPosition(hoodPos);
            }

            if (gamepad1.b) {
                swivel.setPosition(swivelPos);
            }
        }
    }


}

