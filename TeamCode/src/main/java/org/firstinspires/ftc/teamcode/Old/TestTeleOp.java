package org.firstinspires.ftc.teamcode.Old;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp
public class TestTeleOp extends OpMode {
    private DcMotorEx flywheel;
    private DcMotor feedRoller;
    private DcMotor leftDrive;
    private CRServo agitator;
    private DcMotor rightDrive;

    private float flyWheelVelocity = 1300;
    private static final int bankVelocity = 1300;
    private static final int farVelocity = 1900;
    private static final int maxVelocity = 2200;

    private boolean flyWheelPowered;
    private boolean agitatorPowered;
    private boolean feedRollerPowered;

    public void init() {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        feedRoller = hardwareMap.get(DcMotor.class, "coreHex");
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        agitator = hardwareMap.get(CRServo.class, "servo");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");

        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        feedRoller.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        agitator.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("a to turn on/off the flywheel");
        telemetry.addLine("b to turn on/off the agitator");
        telemetry.addLine("x to turn on/off the feed roller");
        telemetry.addLine("Left bumper for slow speed");
        telemetry.addLine("right bumper for medium speed");
        telemetry.update();
    }

    public void loop() {
        basicMovement();
        turnOnMotors();
        flyWheel();
    }

    // -----------------------------------------------------------
    // VOLTAGE SENSOR FUNCTION
    // -----------------------------------------------------------
    private double getVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double v = sensor.getVoltage();
            if (v > 0) {
                result = Math.min(result, v);
            }
        }
        return result;
    }

    public void basicMovement() {
        float x = gamepad1.right_stick_x;
        float y = gamepad1.left_stick_y;
        leftDrive.setPower(y - x);
        rightDrive.setPower(y + x);
    }

    public void turnOnMotors() {
        if(gamepad1.aWasPressed()) {
            flyWheelPowered = !flyWheelPowered;
        }

        if(gamepad1.bWasPressed()) {
            agitatorPowered = !agitatorPowered;
            agitator.setPower(agitatorPowered ? 1 : 0);
        }

        if(gamepad1.xWasPressed()) {
            feedRollerPowered = !feedRollerPowered;
            feedRoller.setPower(feedRollerPowered ? 1 : 0);
        }

        if(gamepad1.y) {
            feedRoller.setPower(-0.5);
        }
        if(gamepad1.yWasReleased()) {
            feedRoller.setPower(0);
        }

        if(gamepad1.leftBumperWasPressed()) {
            flyWheelVelocity = bankVelocity;
        }
        if (gamepad1.rightBumperWasPressed()) {
            flyWheelVelocity = farVelocity;
        }
    }

    // -----------------------------------------------------------
    // FLYWHEEL WITH VOLTAGE COMPENSATION
    // -----------------------------------------------------------
    public void flyWheel() {

        if(flyWheelPowered) {

            // measure current battery voltage
            double voltage = getVoltage();

            // compute multiplier to normalize to 12V
            double multiplier = 14 / voltage;

            // apply it to the commanded velocity
            double compensatedVelocity = flyWheelVelocity * multiplier;

            // send to motor
            flywheel.setVelocity(compensatedVelocity);

            // debug
            telemetry.addData("Battery Voltage", voltage);
            telemetry.addData("Flywheel Target", flyWheelVelocity);
            telemetry.addData("Compensated", compensatedVelocity);

        } else {
            flywheel.setVelocity(0);
        }

        telemetry.update();
    }
}
