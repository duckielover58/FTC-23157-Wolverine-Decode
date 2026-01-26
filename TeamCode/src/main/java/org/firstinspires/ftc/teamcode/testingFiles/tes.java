package org.firstinspires.ftc.teamcode.testingFiles;


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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.driveClasses.PinpointDrive;



@Disabled
@Config
@TeleOp(name = "tes", group = "Robot")
public class tes extends LinearOpMode {

    public long swivelWaitTime = 400;
    public long skep = 100;

    @Override
    public void runOpMode() throws InterruptedException {

        Swivel swivel = new Swivel(hardwareMap);


        waitForStart();
        Gamepad cG1 = new Gamepad();
        Gamepad pG1 = new Gamepad();

        while (opModeIsActive()) {
            pG1.copy(cG1);
            cG1.copy(gamepad1);

            if (cG1.y && !pG1.y) {
                swivelWaitTime += 100;
            }
            if (cG1.a && !pG1.a) {
                swivelWaitTime -= 100;
            }

            if (cG1.b && !pG1.b) {
                Actions.runBlocking(new SequentialAction(swivel.turn90right()));
                sleep(swivelWaitTime);
                Actions.runBlocking(new SequentialAction(swivel.stop()));
                sleep(skep);
            }

            if (cG1.x && !pG1.x) {
                Actions.runBlocking(new SequentialAction(swivel.turn90left()));
                sleep(swivelWaitTime);
                Actions.runBlocking(new SequentialAction(swivel.stop()));
                sleep(skep);
            }

            telemetry.addData("wait time: ", swivelWaitTime);
            telemetry.update();
        }
    }

}