package org.firstinspires.ftc.teamcode.testingFiles;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "IntakePushIndex")
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

        hood.setPosition(hoodPos);
        swivel.setPosition(swivelPos);

        waitForStart();

        while (opModeIsActive()) {



            if (gamepad1.right_bumper) {
                flywheel.setPower(1.0);
                telemetry.addLine("Intake powered");
                telemetry.update();
            }
            else if (gamepad1.left_bumper) {
                flywheel.setPower(0.0);
                telemetry.addLine("Intake turned off");
                telemetry.update();
            }
// very cool comment for pushing
            if (gamepad1.a) {
                onHood = true;
                onSwivel = false;
            }
            if (gamepad1.b) {
                onSwivel = true;
                onHood = false;
            }
            if (gamepad1.dpad_up) {
                if (onHood) hoodPos += step;
                if (onSwivel) swivelPos += step;
            }
            if (gamepad1.dpad_down) {
                if (onHood) hoodPos -= step;
                if (onSwivel) swivelPos -= step;
            }
            if (hoodPos != lastHoodPos) {
                telemetry.addData("New push position: ", hoodPos);
                telemetry.update();
                lastHoodPos = hoodPos;
            }
            if (swivelPos != lastSwivelPos) {
                telemetry.addData("New index position: ", swivelPos);
                telemetry.update();
                lastSwivelPos = swivelPos;
            }

            hood.setPosition(hoodPos);
            swivel.setPosition(swivelPos);

            telemetry.addData("Flywheel Power", flywheel.getPower());
            telemetry.addData("Hood Pos", hoodPos);
            telemetry.addData("Swivel Pos", swivelPos);
            telemetry.update();
        }
    }


}

