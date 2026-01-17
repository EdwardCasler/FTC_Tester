package org.firstinspires.ftc.teamcode.Old;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class FlywheelAutoAdjust extends LinearOpMode {

    private DcMotorEx flywheel;
    private DcMotor feedRoller;
    private CRServo agitator;
    double flywheelTarget = 900; // starting velocity
    double multiplier;
    ElapsedTime timer = new ElapsedTime();

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

        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        feedRoller.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        boolean sequenceStarted = false;

        while (opModeIsActive()) {

            // --- Adjust flywheel velocity with triggers ---
            if (gamepad1.right_trigger > 0.1) {
                flywheelTarget += 5; // increase target
            }
            if (gamepad1.left_trigger > 0.1) {
                flywheelTarget -= 5; // decrease target
            }

            multiplier = 14 / getLowestVoltage();
            flywheel.setVelocity(flywheelTarget * multiplier);

            // --- Start the automatic shooting sequence with a button ---
            if (gamepad1.a && !sequenceStarted) { // press 'A' to start
                sequenceStarted = true;
                timer.reset();

                // Wait 1 second for flywheel to spin up
                while (timer.milliseconds() < 1000 && opModeIsActive()) {
                    flywheel.setVelocity(flywheelTarget * multiplier);
                    idle();
                }

                // Turn on agitator for 0.5 seconds
                agitator.setPower(1);
                timer.reset();
                while (timer.milliseconds() < 500 && opModeIsActive()) {
                    flywheel.setVelocity(flywheelTarget * multiplier);
                    idle();
                }

                // Turn on feed roller for 6 seconds
                feedRoller.setPower(1);
                timer.reset();
                while (timer.milliseconds() < 6000 && opModeIsActive()) {
                    flywheel.setVelocity(flywheelTarget * multiplier);
                    agitator.setPower(1);
                    idle();
                }

                // Stop everything
                flywheel.setVelocity(0);
                agitator.setPower(0);
                feedRoller.setPower(0);
            }

            // --- Telemetry ---
            telemetry.addData("Flywheel Target", flywheelTarget);
            telemetry.addData("Flywheel Velocity", flywheel.getVelocity());
            telemetry.addData("Multiplier", multiplier);
            telemetry.update();
        }
    }
}
