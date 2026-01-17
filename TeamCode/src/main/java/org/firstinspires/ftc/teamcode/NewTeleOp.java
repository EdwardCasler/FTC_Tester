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
public class NewTeleOp extends OpMode {
    private DcMotorEx flywheel;
    private DcMotor feedRoller;
    private DcMotor leftDrive;
    private CRServo agitator;
    private DcMotor rightDrive;

    private float flyWheelVelocity = 1300;
    private float  ticksPerRev = 288;
    private float offset = 0;
    private float FeedRollerSpeed = 65;
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

        feedRoller.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        feedRoller.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheel.setDirection(DcMotor.Direction.REVERSE);
        feedRoller.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        agitator.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addLine("a to turn on/off the flywheel");
        telemetry.addLine("b to turn on/off the agitator");
        telemetry.addLine("x to turn on/off the feed roller");
        telemetry.addLine("y to turn off flywheel agitator and set feed roller angle");

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
            if(!feedRoller.isBusy()) {
                if(feedRollerPowered) {
                    feedRoller.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    feedRollerPowered = false;
                    feedRoller.setPower(0);
                } else {
                    feedRoller.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    feedRollerPowered = true;
                    feedRoller.setPower(FeedRollerSpeed);
                }
            }
        }
        if(gamepad1.yWasPressed()) {
            float angleOff = (feedRoller.getCurrentPosition() % ticksPerRev);

            feedRollerPowered = false;

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
