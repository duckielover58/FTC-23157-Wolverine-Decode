package org.firstinspires.ftc.teamcode.testingFiles;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class LimelightAprilTag extends OpMode {

    private Limelight3A limelight3A;
    private IMU imu;

    @Override
    public void init(){
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(8); //tbd need to change dpdening on april tag pipeline (i think its 0, 1, or 2)
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }

    @Override
    public void start(){
        limelight3A.start();
    }

    @Override
    public void loop(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight3A.updateRobotOrientation(orientation.getYaw());
        LLResult llResult = limelight3A.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botpose = llResult.getBotpose_MT2(); //if not using imu remove _MT2 (metatag 2 so its default metatag 1)
            telemetry.addData("Tx", llResult.getTx()); //target x
            telemetry.addData("Ty", llResult.getTy()); //target y
            telemetry.addData("Ta", llResult.getTa()); //target area



        }
    }


}
