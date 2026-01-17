package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.config.Config;


@TeleOp
@Config
public class OurTeleOp extends OpMode {
    private DcMotorEx flywheel;
    private DcMotor feedRoller;
    private DcMotor leftDrive;
    private CRServo agitator;
    private DcMotor rightDrive;

    public static float flyWheelVelocity = 1300;
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

        // Establishing the direction and mode for the motors
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
        telemetry.addLine("This will disappear when the first interaction begins");
        telemetry.addLine("Voltage control is on");
 
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.update();
    }

    public void loop() {
        basicMovement();
        turnOnMotors();
        flyWheel();

        // Add telemetry data for dashboard
        telemetry.addData("Flywheel Velocity", flywheel.getVelocity());
        telemetry.addData("Flywheel Target", flyWheelVelocity);
        telemetry.addData("Flywheel Powered", flyWheelPowered);
        telemetry.addData("Agitator Powered", agitatorPowered);
        telemetry.addData("Feed Roller Powered", feedRollerPowered);
        telemetry.addData("Left Drive Power", leftDrive.getPower());
        telemetry.addData("Right Drive Power", rightDrive.getPower());
        telemetry.addData("Voltage", getLowestVoltage());

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
        if(gamepad1.aWasPressed()) {
            flyWheelPowered = !flyWheelPowered;
        }
        if(gamepad1.bWasPressed()) {
            if(agitatorPowered) {
                agitatorPowered = false;
                agitator.setPower(0);
            } else {
                agitatorPowered = true;
                agitator.setPower(1);
            }
        }
        if(gamepad1.xWasPressed()) {
            if(feedRollerPowered) {
                feedRollerPowered = false;
                feedRoller.setPower(0);
            } else {
                feedRollerPowered = true;
                feedRoller.setPower(1);
            }
        }
        if(gamepad1.y) {
            feedRoller.setPower(-0.5);
        }
        if(gamepad1.yWasReleased()) {
            feedRoller.setPower(0);
        }
    }
    public void flyWheel() {
        if(flyWheelPowered) {
            double multiplier = 14 / getLowestVoltage();
            flywheel.setVelocity(flyWheelVelocity * multiplier);
        } else {
            flywheel.setVelocity(0);
        }
    }
}
