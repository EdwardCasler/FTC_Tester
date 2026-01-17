package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class BlueAutoWithShooting extends LinearOpMode {
    private DcMotorEx flywheel;
    private DcMotor feedRoller;
    private CRServo agitator;
    private DcMotor leftDrive;
    private DcMotor rightDrive;

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
        // Flywheel/shooter hardware
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        feedRoller = hardwareMap.get(DcMotor.class, "coreHex");
        agitator = hardwareMap.get(CRServo.class, "servo");

        // Drivetrain hardware
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        feedRoller.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();

        // --- Shooting sequence ---
        multiplier = 14 / getLowestVoltage();
        ourSleep(1000);           // Spin up flywheel
        //agitator.setPower(1);
        //ourSleep(500);            // Agitate
        feedRoller.setPower(1);
        ourSleep(6000);           // Feed balls

        // Stop shooter motors
        flywheel.setPower(0);
        //agitator.setPower(0);
        feedRoller.setPower(0);

        rightDrive.setPower(-1);
        leftDrive.setPower(1);
        sleep(200);
        leftDrive.setPower(1);
        rightDrive.setPower(1);
        sleep(700);
        leftDrive.setPower(0);
        rightDrive.setPower(0);

    }

    void ourSleep(double timeTakes) {
        time.reset();
        while (time.milliseconds() < timeTakes && opModeIsActive()) {
            multiplier = 14 / getLowestVoltage();
            flywheel.setVelocity(900 * multiplier);
            idle();
        }
    }
}