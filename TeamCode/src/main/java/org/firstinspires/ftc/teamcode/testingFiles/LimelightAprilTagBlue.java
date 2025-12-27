package org.firstinspires.ftc.teamcode.testingFiles;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;


@TeleOp
public class LimelightAprilTagBlue extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException
    {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");


        waitForStart();

        while (opModeIsActive()) {
            limelight.start();
            limelight.pipelineSwitch(0);
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double distance = (75-33) / Math.tan(Math.toRadians((90-74.027037)+result.getTy()));
                double corrected = distance + 0.4 * (distance - 130) - 10;
                Pose3D botpose = result.getBotpose();
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("ta", result.getTa());
                telemetry.addData("Botpose", botpose.toString());
                telemetry.addData("distance: ", corrected);
            } else {
                telemetry.addLine("no result");
            }
            telemetry.update();
            sleep(20); // prevent overloading the loop
            limelight.stop();
        }
    }
}
