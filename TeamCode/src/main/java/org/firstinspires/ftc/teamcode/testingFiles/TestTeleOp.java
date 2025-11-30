package org.firstinspires.ftc.teamcode.testingFiles;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Index;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Push;
import org.firstinspires.ftc.teamcode.subsystems.Swivel;

@TeleOp(name = "TestTeleOp")
public class TestTeleOp extends LinearOpMode {

    private boolean pushedUp = false;

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    Intake intake;
    Push push;
    Flywheel flywheel;
    Swivel swivel;
    Index index;

    @Override
    public void runOpMode() {

        // --- INIT MOTORS ---
        leftFront  = hardwareMap.get(DcMotor.class, "FrontLeft");
        rightFront = hardwareMap.get(DcMotor.class, "FrontRight");
        leftBack   = hardwareMap.get(DcMotor.class, "BackLeft");
        rightBack  = hardwareMap.get(DcMotor.class, "BackRight");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        // --- INIT SUBSYSTEMS ---
        intake = new Intake(hardwareMap);
        push = new Push(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        swivel = new Swivel(hardwareMap);
        index = new Index(hardwareMap);

        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            double vertical   = -gamepad1.left_stick_y;
            double horizontal = gamepad1.left_stick_x;
            double pivot      = gamepad1.right_stick_x;

            double frontLeftPower  = vertical + horizontal + pivot;
            double frontRightPower = vertical + horizontal - pivot;
            double backLeftPower   = vertical - horizontal + pivot;
            double backRightPower  = vertical - horizontal - pivot;

            double max = Math.max(Math.abs(frontLeftPower),
                    Math.max(Math.abs(frontRightPower),
                            Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));

            if (max > 1.0) {
                frontLeftPower  /= max;
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }

            leftFront.setPower(frontLeftPower);
            rightFront.setPower(frontRightPower);
            leftBack.setPower(backLeftPower);
            rightBack.setPower(backRightPower);


            if (gamepad1.a) {
                Actions.runBlocking(intake.IntakeBall());
                sleep(500);
            }
            if (gamepad1.left_bumper) {
                Actions.runBlocking(intake.IntakeBallStop());
                sleep(500);
            }
            if (gamepad1.b) {
                Actions.runBlocking(push.PushBallUp());
                sleep(500);
            }
            if (gamepad1.dpad_down) {
                Actions.runBlocking(push.PushBallDown());
                sleep(500);
            }
/*
            if (gamepad1.b && !pushedUp) {
                Actions.runBlocking(push.PushBallUp());
                sleep(100);
                pushedUp = true;
            }
            else if (pushedUp == true) {
                Actions.runBlocking(push.PushBallDown());
                sleep(100);
                pushedUp = false;
            }

 */

            if (gamepad1.y) {
                Actions.runBlocking(flywheel.shoot());
                sleep(500);
            }
            if (gamepad1.x) {
                Actions.runBlocking(index.index1());
                sleep(500);
            }
            if (gamepad1.dpad_up) {
                Actions.runBlocking(index.index2());
                sleep(500);
            }
            if (gamepad1.dpad_left) {
                Actions.runBlocking(index.index3());
                sleep(500);
            }
            if (gamepad1.dpad_right) {
                Actions.runBlocking(swivel.aim());
                sleep(500);
            }
            if (gamepad1.right_bumper) {
                Actions.runBlocking(flywheel.shootStop());
                sleep(500);
            }

            telemetry.update();
        }
    }
}
