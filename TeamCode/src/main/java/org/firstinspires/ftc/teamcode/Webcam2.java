package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class Webcam2 extends OpMode {
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    float maxForwardSpeed = 0.5f;
    float maxStrafeSpeed = 0.5f;
    float maxYawSpeed = 0.5f;
    float forwardGain = 0.02f;
    float strafeGain = 0.015f;
    float yawGain = 0.01f;
    float distToBe = 25; //inches

    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    public void init() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "fl");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "fr");
        leftBackDrive = hardwareMap.get(DcMotor.class, "bl");
        rightBackDrive = hardwareMap.get(DcMotor.class, "br");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        aprilTagWebcam.init(hardwareMap, telemetry);
    }

    public void loop() {
        aprilTagWebcam.update();
        AprilTagDetection id20 = aprilTagWebcam.getTagById(20);
        aprilTagWebcam.displayDetectionTelemetry(id20);

        double forwardError = id20.ftcPose.range - distToBe;
        double yawError = -id20.ftcPose.yaw;  //tags rotation relative to camera.
        double turnError = id20.ftcPose.bearing; //This is angle of the center of the tag in the frame
        double forward = Range.clip(forwardError * forwardGain, -maxForwardSpeed, maxForwardSpeed);
        double strafe = Range.clip(yawError * strafeGain, -maxStrafeSpeed, maxStrafeSpeed);
        double turn = Range.clip(turnError * yawGain, -maxYawSpeed, maxYawSpeed);
        drive((float)forward, (float)strafe, (float)turn);
    }
    void drive(float forward, float strafe, float rotate) {
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        leftFrontDrive.setPower((forward + strafe + rotate) / denominator);
        rightFrontDrive.setPower((forward - strafe - rotate) / denominator);
        leftBackDrive.setPower((forward - strafe + rotate) / denominator);
        rightBackDrive.setPower((forward + strafe - rotate) / denominator);
    }
}