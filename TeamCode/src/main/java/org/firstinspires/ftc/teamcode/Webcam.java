package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous
public class Webcam extends OpMode {
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);

        leftFrontDrive = hardwareMap.get(DcMotor.class, "fl");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "fr");
        leftBackDrive = hardwareMap.get(DcMotor.class, "bl");
        rightBackDrive = hardwareMap.get(DcMotor.class, "br");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
    }
    public void loop() {
        aprilTagWebcam.update();
        AprilTagDetection id20 = aprilTagWebcam.getTagById(20);
        aprilTagWebcam.displayDetectionTelemetry(id20);

        //move to apriltag
        if(id20 != null) {
            //if distances are close enough don't care
            float xDist = Math.abs(id20.ftcPose.x) > 2 ? (float)id20.ftcPose.x : 0;
            float zDist = Math.abs(id20.ftcPose.z) > 5 ? (float)id20.ftcPose.z : 0;
            float yError = Math.abs(id20.ftcPose.yaw) > 3 ? (float)id20.ftcPose.yaw : 0;

            float x = Math.signum(xDist) * 0.25f;
            float y = Math.signum(yError) * 0.25f;
            float z = Math.signum(zDist) * 0.25f;

            if(id20.ftcPose.range > 20) {
                drive(z, x, 0);
                //drive(0, 0, y);
            }
        }
    }
    void drive(float forward, float strafe, float rotate) {
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        leftFrontDrive.setPower((forward + strafe + rotate) / denominator);
        rightFrontDrive.setPower((forward - strafe - rotate) / denominator);
        leftBackDrive.setPower((forward - strafe + rotate) / denominator);
        rightBackDrive.setPower((forward + strafe - rotate) / denominator);
    }
}
