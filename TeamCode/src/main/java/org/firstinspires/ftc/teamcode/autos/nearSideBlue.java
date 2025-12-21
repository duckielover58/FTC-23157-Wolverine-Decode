package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.nearBlue.Intake3X;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.nearBlue.Intake3Y;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.nearBlue.IntakeEnd3X;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.nearBlue.IntakeEnd3Y;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.nearBlue.shootH;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.nearBlue.shootShortX;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.nearBlue.shootShortY;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.nearBlue.shootLongX;
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
import static org.firstinspires.ftc.teamcode.testingFiles.Flywheelgm0PIDtest.targetVelocity;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
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

    @Override
    public void runOpMode() throws InterruptedException {
        flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        Intake intake = new Intake(hardwareMap);
        Hood hood = new Hood(hardwareMap);
        Index index = new Index(hardwareMap);
        Push push = new Push(hardwareMap);
        Swivel swivel = new Swivel(hardwareMap);
        Pose2d startingPose = new Pose2d(startX, startY, startH);

        PinpointDrive drive = new PinpointDrive(hardwareMap, startingPose);

        waitForStart();

        Action tab1 = drive.actionBuilder(startingPose)

                .lineToX(shootLongX)
                //Shoot

                //Intake 1st spike mark
                .stopAndAdd(new SequentialAction(intake.IntakeBall()))
                .setTangent(-20)
                .splineToLinearHeading(new Pose2d(Intake1X, Intake1Y, IntakeH), Math.toRadians(270))
                .afterTime(.2, new SequentialAction(index.turnRight()))
                .afterTime(.2, new SequentialAction(index.turnRight()))
                .afterTime(.2, new SequentialAction(index.turnRight()))
                .stopAndAdd(new SequentialAction(intake.IntakeBallStop()))

                //Shoot short
                .strafeToLinearHeading(new Vector2d(shootShortX, shootShortY), shootH)
                //Shoot


                //Intake 2nd spike mark
                .stopAndAdd(new SequentialAction(intake.IntakeBall()))
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(Intake2X, Intake2Y, IntakeH), Math.toRadians(270))
                .afterDisp(3, new SequentialAction(index.turnRight()))
                .afterDisp(3, new SequentialAction(index.turnRight()))
                .afterDisp(3, new SequentialAction(index.turnRight()))
                .stopAndAdd(new SequentialAction(intake.IntakeBallStop()))

                //Shoot short
                .setTangent(90)
                .strafeToLinearHeading(new Vector2d(shootShortX, shootShortY), shootH)
                //Shoot

                .stopAndAdd(new SequentialAction(intake.IntakeBall()))
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(Intake3X, Intake3Y, IntakeH), Math.toRadians(0))
                .strafeTo(new Vector2d(IntakeEnd3X, IntakeEnd3Y))
                .afterDisp(3, new SequentialAction(index.turnRight()))
                .afterDisp(3, new SequentialAction(index.turnRight()))
                .afterDisp(3, new SequentialAction(index.turnRight()))
                .stopAndAdd(new SequentialAction(intake.IntakeBallStop()))
                .strafeToLinearHeading(new Vector2d(shootShortX, shootShortY), shootH)
                .build();

        Actions.runBlocking(tab1);
    }

//    DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");


    void flywheelPID (double target) {

        double currentVelocity = flywheel.getVelocity();
        double error = targetVelocity - currentVelocity;

        double ff = kF * targetVelocity;

        double output = ff + (kP * error);

        output = Math.max(0.0, Math.min(1.0, output));

        flywheel.setPower(output);
    }
}

