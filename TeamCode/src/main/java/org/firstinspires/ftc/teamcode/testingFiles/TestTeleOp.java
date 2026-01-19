package org.firstinspires.ftc.teamcode.testingFiles;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.driveClasses.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Index;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Push;
import org.firstinspires.ftc.teamcode.subsystems.Swivel;

@Disabled
@TeleOp(name = "TestTeleOp")
public class TestTeleOp extends OpMode {

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    Intake intake;
    Push push;
    Flywheel flywheel;
    Swivel swivel;
    Index index;
    Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");


    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotor.class, "FrontLeft");
        rightFront = hardwareMap.get(DcMotor.class, "FrontRight");
        leftBack = hardwareMap.get(DcMotor.class, "BackLeft");
        rightBack = hardwareMap.get(DcMotor.class, "BackRight");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        intake = new Intake(hardwareMap);
        push = new Push(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        swivel = new Swivel(hardwareMap);
        index = new Index(hardwareMap);

    }

    public void start() {
        Actions.runBlocking(index.indexHome());
    }


    private class ShootThreeBallsPPG implements Action {
        private final Action sequence;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            return sequence.run(packet);
        }

        public ShootThreeBallsPPG() {
            sequence = new SequentialAction(
                    index.intakeIndex2(),
                    flywheel.shoot(),
                    new SleepAction(0.2),
                    push.PushBallDown(),
                    new SleepAction(4.0),
                    push.PushBallUp(),
                    new SleepAction(0.3),
                    push.PushBallDown(),
                    new SleepAction(0.5),


                    index.intakeIndex3(),
                    new SleepAction(0.85),
                    push.PushBallUp(),
                    new SleepAction(0.3),
                    push.PushBallDown(),
                    new SleepAction(0.5),


                    index.intakeIndex1(),
                    new SleepAction(1.0),
                    push.PushBallUp(),
                    new SleepAction(0.3),
                    push.PushBallDown(),
                    new SleepAction(0.5),
                    flywheel.shootStop(),
                    new SleepAction(0.2),
                    index.intakeIndex1()
            );
        }
    }
//hood angle at 2
    //hello

    @Override
    public void loop() {

        double vertical = -gamepad1.left_stick_y;
        double horizontal = gamepad1.left_stick_x;
        double pivot = gamepad1.right_stick_x;

        double frontLeftPower = vertical + horizontal + pivot;
        double frontRightPower = vertical + horizontal - pivot;
        double backLeftPower = vertical - horizontal + pivot;
        double backRightPower = vertical - horizontal - pivot;

        double max = Math.max(Math.abs(frontLeftPower),
                Math.max(Math.abs(frontRightPower),
                        Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        leftFront.setPower(frontLeftPower);
        rightFront.setPower(frontRightPower);
        leftBack.setPower(backLeftPower);
        rightBack.setPower(backRightPower);

        if (gamepad1.a) {
            Actions.runBlocking(intake.IntakeBall());
        }
        if (gamepad1.left_bumper) {
            Actions.runBlocking(intake.IntakeBallStop());
        }
        if (gamepad1.b) {
            Actions.runBlocking(push.PushBallUp());
        }
        if (gamepad1.dpad_down) {
            Actions.runBlocking(push.PushBallDown());
        }
        if (gamepad1.y) {
            Actions.runBlocking(flywheel.shoot());
        }
        if (gamepad1.right_bumper) {
            Actions.runBlocking(flywheel.shootStop());
        }
        if (gamepad1.x) {
            Actions.runBlocking(index.intakeIndex1());
        }
        if (gamepad1.dpad_up) {
            Actions.runBlocking(index.intakeIndex2());
        }
        if (gamepad1.dpad_left) {
            Actions.runBlocking(index.intakeIndex3());
        }
        if (gamepad1.dpad_right) {
            Actions.runBlocking(new ShootThreeBallsPPG());
        }
        if (gamepad2.right_bumper) {
            Actions.runBlocking(swivel.aim());
        }
        if (gamepad2.a) {
            limelight.stop();
        }
    }
}
