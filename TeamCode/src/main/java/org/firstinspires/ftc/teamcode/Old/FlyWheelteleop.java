package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.VoltageSensor;


@TeleOp
public class FlyWheelteleop extends OpMode {
    private DcMotorEx flywheel;
    private DcMotor feedRoller;
    private DcMotor leftDrive;
    private CRServo agitator;
    private DcMotor rightDrive;

    private float flyWheelVelocity = 1300;
    private float  ticksPerRev = 288;
    private float offset = 0;
    // private float FeedRollerSpeed = 65; // No longer needed
    private boolean flyWheelPowered;
    private boolean agitatorPowered;
    // private boolean feedRollerPowered; // No longer needed

    public void init() {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        feedRoller = hardwareMap.get(DcMotor.class, "coreHex");
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        agitator = hardwareMap.get(CRServo.class, "servo");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");

        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        feedRoller.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        feedRoller.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheel.setDirection(DcMotor.Direction.REVERSE);
        feedRoller.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        agitator.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addLine("a to turn on/off the flywheel");
        telemetry.addLine("b to turn on/off the agitator");
        telemetry.addLine("RIGHT/LEFT TRIGGERS to run the feed roller");
        telemetry.addLine("y to turn off flywheel/agitator and reset feed roller");

        telemetry.update();
    }

    public void loop() {
        basicMovement();
        turnOnMotors();
        flyWheel();

        telemetry.addLine("Encoder Position: " + String.valueOf(feedRoller.getCurrentPosition()));
        telemetry.addLine("Offset: " + offset);
        telemetry.update();
    }
    public double getLowestVoltage() {
        double lowestValue = Double.POSITIVE_INFINITY;
        for(VoltageSensor sensor : hardwareMap.voltageSensor) {
            if(sensor.getVoltage() < lowestValue && sensor.getVoltage() > 0.1) {
                lowestValue = sensor.getVoltage();
            }
        }
        if(lowestValue == Double.POSITIVE_INFINITY) {
            lowestValue = 14;
        }
        telemetry.addLine("Voltage: " + lowestValue + "V");
        return lowestValue;
    }
    public void basicMovement() {
        float x;
        float y;

        x = gamepad1.right_stick_x;
        y = gamepad1.left_stick_y;
        leftDrive.setPower(y - x);
        rightDrive.setPower(y + x);
    }

    public void turnOnMotors() {
        // --- Flywheel (A button) ---
        if(gamepad1.aWasPressed()) {
            flyWheelPowered = !flyWheelPowered;
        }

        // --- Agitator (B button) ---
        if(gamepad1.bWasPressed()) {
            if(agitatorPowered) {
                agitatorPowered = false;
                agitator.setPower(0);
            } else {
                agitatorPowered = true;
                agitator.setPower(1);
            }
        }

        // --- Feed Roller (Triggers) ---
        // Get trigger values (0.0 to 1.0)
        float forwardPower = gamepad1.right_trigger;
        float reversePower = gamepad1.left_trigger;

        // This check is important! It stops the triggers from fighting the 'Y' button command.
        if (!feedRoller.isBusy()) {
            // Ensure we are in the correct mode to accept power commands
            feedRoller.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            if (forwardPower > 0.1) {
                // Right trigger pressed: run forward
                // Your motor direction is REVERSE, so a positive power makes it run "reverse".
                // If this is the wrong way, just make this -forwardPower
                feedRoller.setPower(forwardPower);
            } else if (reversePower > 0.1) {
                // Left trigger pressed: run in reverse
                // A negative power will make it run "forward" (opposite of REVERSE)
                feedRoller.setPower(-reversePower);
            } else {
                // No trigger pressed: stop the motor
                feedRoller.setPower(0);
            }
        }

        // --- All Stop / Reset (Y button) ---
        if(gamepad1.yWasPressed()) {
            float angleOff = (feedRoller.getCurrentPosition() % ticksPerRev);

            // feedRollerPowered is no longer needed
            // feedRollerPowered = false;

            feedRoller.setTargetPosition((int)(feedRoller.getCurrentPosition() - angleOff));
            feedRoller.setPower(1);
            feedRoller.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            flywheel.setPower(0);
            agitator.setPower(0);
        }
    }

    public void flyWheel() {
        if(flyWheelPowered) {
            double multiplier = 14 / getLowestVoltage();
            telemetry.addData("Velocity", flyWheelVelocity * multiplier);
            flywheel.setVelocity(flyWheelVelocity * multiplier);
        } else {
            flywheel.setVelocity(0);
        }
    }
}