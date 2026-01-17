package org.firstinspires.ftc.teamcode.Old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class AutoWithMove extends LinearOpMode {

    private DcMotorEx flywheel;
    private DcMotor feedRoller;
    private CRServo agitator;
    private DcMotor leftDrive, rightDrive; // drivetrain motors
    float ticksPerRev = 28 * 5 * 3; // 5 and 3 is the gear reductions
    float circumference = 9.5f; // in inches
    float ticksPerInch = ticksPerRev / circumference;
    ElapsedTime time = new ElapsedTime();
    double multiplier;

    public double getLowestVoltage() {
        double lowestValue = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            if (sensor.getVoltage() < lowestValue && sensor.getVoltage() > 0.1) {
                lowestValue = sensor.getVoltage();
            }
        }
        if (lowestValue == Double.POSITIVE_INFINITY) {
            lowestValue = 14;
        }
        return lowestValue;
    }

    @Override
    public void runOpMode() {

        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        feedRoller = hardwareMap.get(DcMotor.class, "coreHex");
        agitator = hardwareMap.get(CRServo.class, "servo");

        // **Add drivetrain motors**
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");

        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        feedRoller.setDirection(DcMotorSimple.Direction.FORWARD);
        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD); // adjust if necessary

        // Reset and run flywheel using encoders
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        // --- Back up 1 foot before starting sequence ---
        moveBackward(12); // 12 inches = 1 ft

        // --- Shooting sequence ---
        multiplier = 14 / getLowestVoltage();
        ourSleep(1000);
        agitator.setPower(1);
        ourSleep(500);
        feedRoller.setPower(1);
        ourSleep(6000);

        // Stop everything
        flywheel.setPower(0);
        agitator.setPower(0);
        feedRoller.setPower(0);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    // --- Helper method to move backward ---
    void moveBackward(double inches) {
        int targetTicks = (int)(inches * ticksPerInch);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setTargetPosition(-targetTicks);  // negative for backward
        rightDrive.setTargetPosition(-targetTicks);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(0.5);
        rightDrive.setPower(0.5);

        while (opModeIsActive() && leftDrive.isBusy() && rightDrive.isBusy()) {
            idle();
        }

        // Stop motors
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        // Return to RUN_USING_ENCODER for teleop/autonomous control
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void ourSleep(double timeTakes) {
        time.reset();
        while (time.milliseconds() < timeTakes && opModeIsActive()) {
            multiplier = 14 / getLowestVoltage();
            flywheel.setVelocity(900 * multiplier);

            telemetry.addLine("Multiplier: " + multiplier);
            telemetry.addLine(String.valueOf(flywheel.getVelocity()));
            telemetry.update();
            idle();
        }
    }
}
