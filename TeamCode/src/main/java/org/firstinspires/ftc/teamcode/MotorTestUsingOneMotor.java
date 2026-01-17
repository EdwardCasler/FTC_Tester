package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class MotorTestUsingOneMotor extends LinearOpMode {
    private DcMotor motor1;

    @Override
    public void runOpMode() {

        motor1 = hardwareMap.get(DcMotor.class, "motor");

        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized with Encoders");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double stick = -gamepad1.right_stick_y;
            motor1.setPower(stick);

            telemetry.addData("Right Stick", stick);
            telemetry.addData("Motor Power", motor1.getPower());

            telemetry.addData("Encoder Position", motor1.getCurrentPosition());

            telemetry.update();
        }

        motor1.setPower(0);
    }
}