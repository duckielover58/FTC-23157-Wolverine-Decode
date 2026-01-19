package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.nearBlue.Intake3X;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.nearBlue.Intake3Y;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.nearBlue.IntakeEnd3X;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.nearBlue.IntakeEnd3Y;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.nearBlue.shootH;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.nearBlue.shootShortX;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.nearBlue.shootShortY;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.nearBlue.startH;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.nearBlue.IntakeH;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.nearBlue.Intake1X;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.nearBlue.Intake1Y;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.nearBlue.Intake2X;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.nearBlue.Intake2Y;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.nearBlue.startX;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.nearBlue.startY;
import static org.firstinspires.ftc.teamcode.testingFiles.Flywheelgm0PIDtest.kF;
import static org.firstinspires.ftc.teamcode.testingFiles.Flywheelgm0PIDtest.kP;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.subsystems.GlobalVariable;

// Non-RR imports
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.driveClasses.PinpointDrive;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Index;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Push;
import org.firstinspires.ftc.teamcode.subsystems.Swivel;

@Config
@Autonomous(name = "nearSideBlue", group = "Robot")
public class nearSideBlue extends LinearOpMode{

    double integralSum = GlobalVariable.integralSum;
    double previousError = GlobalVariable.previousError;
    DcMotorEx flywheel;
    Intake intake = new Intake(hardwareMap);
    Hood hood = new Hood(hardwareMap);
    Index index = new Index(hardwareMap);
    Push push = new Push(hardwareMap);
    Swivel swivel = new Swivel(hardwareMap);

    public static final int RED_TAG_ID = 24;
    private class ShootThreeBalls implements Action {
        private final Action sequence;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            return sequence.run(packet);
        }
/*
        public IntakeThreeBalls() {
            sequence = new SequentialAction(


            )
        }

 */


        public ShootThreeBalls() {
            sequence = new SequentialAction(
                    //first
                    index.intakeIndex2(),
                    new SleepAction(0.2),
                    push.PushBallDown(),
                    new SleepAction(1.3),
                    push.PushBallUp(),


                    //second ball
                    hood.seven(),
                    new SleepAction(0.3),
                    push.PushBallDown(),
                    new SleepAction(0.45),
                    index.intakeIndex3(),
                    new SleepAction(0.25),
                    push.PushBallUp(),

                    //third ball
                    hood.six(),
                    new SleepAction(0.3),
                    push.PushBallDown(),
                    new SleepAction(0.5),
                    index.intakeIndex1(),
                    new SleepAction(0.3),
                    push.PushBallUp(),


                    //wrap up
                    new SleepAction(0.3),
                    new InstantAction(() -> flywheelPID(0)),
                    push.PushBallDown(),
                    index.intakeIndex1()
            );
        }
    }
    private class StartRev implements Action {

        private final Action sequence;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            return sequence.run(packet);
        }

        public StartRev() {
            sequence = new SequentialAction(
                    new InstantAction(() -> flywheelPID(350))
            );
        }
    }
    private class ShootThreeBallsCorner implements Action {
        private final Action sequence;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            return sequence.run(packet);
        }

        public ShootThreeBallsCorner() {
            sequence = new SequentialAction(
                    index.intakeIndex3(),
                    new SleepAction(0.2),
                    push.PushBallDown(),
                    new SleepAction(1.9),
                    push.PushBallUp(),
                    new SleepAction(0.3),
                    push.PushBallDown(),
                    new SleepAction(0.3),
                    index.intakeIndex2()
            );
        }
    }
    private class ShootThreeBallsCornerTwo implements Action {
        private final Action sequence;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            return sequence.run(packet);
        }
        public ShootThreeBallsCornerTwo() {        //hood
            sequence = new SequentialAction(
                    new SleepAction(1.0),
                    push.PushBallUp(),
                    new SleepAction(0.3),
                    push.PushBallDown(),
                    new SleepAction(0.5),
                    index.intakeIndex1(),
                    new SleepAction(0.6),
                    push.PushBallUp(),
                    new SleepAction(0.3),
                    push.PushBallDown(),
                    new SleepAction(0.5),
                    new InstantAction(() -> flywheelPID(0)),
                    new SleepAction(0.2),
                    index.intakeIndex1()
            );
        }
    }
    void flywheelPID (double target) {

        double currentVelocity = flywheel.getVelocity();
        double error = target - currentVelocity;

        double ff = kF * target;

        double output = ff + (kP * error);

        output = Math.max(0.0, Math.min(1.0, output));

        flywheel.setPower(output);
    }


    @Override
    public void runOpMode() throws InterruptedException {
        flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        Pose2d startingPose = new Pose2d(startX, startY, startH);

        PinpointDrive drive = new PinpointDrive(hardwareMap, startingPose);

        Action inits = drive.actionBuilder(startingPose)
            .stopAndAdd(new SequentialAction(
                    push.PushBallDown(),
                    index.intakeIndex1(),
                    hood.hoodPositionInit()
            ))
            .build();

        Action tab1 = drive.actionBuilder(startingPose)
                //Shoot
                .build();
        Action tab2 = drive.actionBuilder(drive.pose)
                //Intake 1st spike mark
                .stopAndAdd(intake.IntakeBall())
                .setTangent(-20)
                .splineToLinearHeading(new Pose2d(Intake1X, Intake1Y, IntakeH), Math.toRadians(270))
                .afterTime(.2, index.intakeIndex1())
                .stopAndAdd(new SleepAction(.2))
                .afterTime(.2, index.intakeIndex2())
                .stopAndAdd(new SleepAction(.2))
                .afterTime(.2, index.intakeIndex3())
                .stopAndAdd(intake.IntakeBallStop())

                //Shoot short
                .strafeToLinearHeading(new Vector2d(shootShortX, shootShortY), shootH)
                //Shoot


                //Intake 2nd spike mark
                .stopAndAdd(intake.IntakeBall())
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(Intake2X, Intake2Y, IntakeH), Math.toRadians(270))
                .afterTime(.2, index.intakeIndex1())
                .afterTime(.2, index.intakeIndex2())
                .afterTime(.2, index.intakeIndex3())
                .stopAndAdd(intake.IntakeBallStop())

                //Shoot short
                .setTangent(90)
                .strafeToLinearHeading(new Vector2d(shootShortX, shootShortY), shootH)
                //Shoot

                .stopAndAdd(intake.IntakeBall())
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(Intake3X, Intake3Y, IntakeH), Math.toRadians(0))
                .strafeTo(new Vector2d(IntakeEnd3X, IntakeEnd3Y))
                .afterTime(.2, index.intakeIndex1())
                .afterTime(.2, index.intakeIndex2())
                .afterTime(.2, index.intakeIndex3())
                .stopAndAdd(intake.IntakeBallStop())

                //Shoot short
                .strafeToLinearHeading(new Vector2d(shootShortX, shootShortY), shootH)
                //shoot
                .build();

        Actions.runBlocking(inits);

        waitForStart();

        Actions.runBlocking(new SequentialAction(tab1, tab2));
    }

//    DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");

}

