package org.firstinspires.ftc.teamcode.autos;


// RR-specific imports
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Index;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Push;
import org.firstinspires.ftc.teamcode.subsystems.Swivel;

// Non-RR imports
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.driveClasses.PinpointDrive;


@Config
@Autonomous(name = "tes", group = "Robot")
public class tes extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Swivel swivel = new Swivel(hardwareMap);

        long swivelWaitTime = 0;
        double inv = 0.1;
        long skep = 100;
        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpad_up) {
                swivelWaitTime += inv;
                sleep(skep);
            }
            if (gamepad1.dpad_down) {
                swivelWaitTime -= inv;
                sleep(skep);
            }

            if (gamepad1.a) {
                Actions.runBlocking(new SequentialAction(swivel.turn90right()));
                sleep(swivelWaitTime);
                Actions.runBlocking(new SequentialAction(swivel.stop()));
                sleep(skep);
            }

            telemetry.addData("wait time: ", swivelWaitTime);
            telemetry.update();
        }
    }

}