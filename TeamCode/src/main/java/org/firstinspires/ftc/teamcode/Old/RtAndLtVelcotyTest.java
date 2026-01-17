package org.firstinspires.ftc.teamcode.Old;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp; // changed to TeleOp
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;


//Made with AI
@TeleOp
public class RtAndLtVelcotyTest extends LinearOpMode {

    private DcMotorEx flywheel;
    private DcMotor feedRoller;
    private CRServo agitator;
    double flywheelTarget = 900; // starting velocity
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

        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        feedRoller.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            // adjust flywheel target using triggers
            if (gamepad1.right_trigger > 0.1) {
                flywheelTarget += 5; // increase
            }
            if (gamepad1.left_trigger > 0.1) {
                flywheelTarget -= 5; // decrease
            }

            // apply voltage compensation
            multiplier = 14 / getLowestVoltage();
            flywheel.setVelocity(flywheelTarget * multiplier);

            // optional telemetry
            telemetry.addData("Flywheel Target", flywheelTarget);
            telemetry.addData("Flywheel Velocity", flywheel.getVelocity());
            telemetry.addData("Multiplier", multiplier);
            telemetry.update();
        }
    }
}
