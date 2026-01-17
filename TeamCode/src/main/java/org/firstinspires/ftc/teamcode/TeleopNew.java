package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.VoltageSensor;

//This is a teleop
@TeleOp
public class TeleopNew extends OpMode {
    private DcMotorEx flywheel;
    private DcMotor feedRoller;
    private DcMotor leftDrive;
    private CRServo agitator;
    private DcMotor rightDrive;
//s
    private float flyWheelVelocity = 1300;
    private float  ticksPerRev = 288;
    private float offset = 0;

    private double feedRollerSpeed = 0.65;
    private final double FEED_ROLLER_INCREMENT = 0.05;
    private final double MAX_FEED_ROLLER_SPEED = 1.0;

    private boolean feedRollerActive = false;
    private boolean flyWheelPowered;
    private boolean agitatorPowered;


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

        // --- UPDATED TELEMETRY ---
        telemetry.addLine("a to turn on/off the flywheel");
        telemetry.addLine("b to turn on/off the agitator");
        telemetry.addLine("X to start/stop the feed roller");
        telemetry.addLine("RIGHT/LEFT BUMPERS to change feed roller speed");
        telemetry.addLine("y to turn off all and reset feed roller");
        telemetry.addLine("");

        telemetry.update();
    }

    public void loop() {
        basicMovement();
        turnOnMotors();
        flyWheel();

        telemetry.addLine("Encoder Position: " + String.valueOf(feedRoller.getCurrentPosition()));
        telemetry.addLine("Offset: " + offset);

        // --- NEW TELEMETRY ---
        telemetry.addLine("Feed Roller Speed: " + String.format("%.2f", feedRollerSpeed));
        telemetry.addLine("Feed Roller Active: " + feedRollerActive);
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

        // --- Feed Roller ON/OFF (X button) ---
        if(gamepad1.xWasPressed()) {
            feedRollerActive = !feedRollerActive;
        }

        // --- Feed Roller Speed Adjustment (Bumpers) ---
        if (gamepad1.rightBumperWasPressed()) {
            feedRollerSpeed += FEED_ROLLER_INCREMENT;
        } else if (gamepad1.leftBumperWasPressed()) {
            feedRollerSpeed -= FEED_ROLLER_INCREMENT;
        }

        // Clamp the speed to make sure it stays between -1.0 (full reverse) and 1.0 (full forward)
        feedRollerSpeed = Math.max(-MAX_FEED_ROLLER_SPEED, Math.min(MAX_FEED_ROLLER_SPEED, feedRollerSpeed));


        // --- All Stop / Reset (Y button) ---
        if(gamepad1.yWasPressed()) {
            float angleOff = (feedRoller.getCurrentPosition() % ticksPerRev);

            // Turn off continuous roller control
            feedRollerActive = false;
            feedRollerSpeed = 0.0;

            // Start RUN_TO_POSITION to reset angle
            feedRoller.setTargetPosition((int)(feedRoller.getCurrentPosition() - angleOff));
            feedRoller.setPower(1.0);
            feedRoller.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            flywheel.setPower(0);
            agitator.setPower(0);
        }

        // --- Apply Power to Feed Roller ---
        if (!feedRoller.isBusy()) {
            feedRoller.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            if (feedRollerActive) {
                // If the roller is active, set it to the adjusted speed
                feedRoller.setPower(feedRollerSpeed);
            } else {
                // If not active, stop it
                feedRoller.setPower(0);
            }
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