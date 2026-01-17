package org.firstinspires.ftc.teamcode.autos.REV;  //Package for autos

//Imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;    //It is a auto
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;                  //Allow us to control motors
import com.qualcomm.robotcore.hardware.DcMotorSimple;            //Allow us to control motors

@Autonomous  //Make this a auto
public class BlueAuto extends LinearOpMode { //Ignore this line, but worry about the "{" symbol. That opens the class in this OOP langauge
    private DcMotor leftDrive;
    private DcMotor rightDrive;

    @Override //Overide the empty code that is default
    public void runOpMode() { //Run the opmode
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");    //This finds the left motor
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");  //This finds the right motor

        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);              //Reverses the motor
        waitForStart();                                 //Wait for the start button to be clicked

        rightDrive.setPower(-1);    //One motor goes backward, one goes forward, so it turns
        leftDrive.setPower(1);
        sleep(400);      //Give it time to turn
        leftDrive.setPower(1);       //Move forward
        rightDrive.setPower(1);       // Move forward
        sleep(700);       //Let it move
        leftDrive.setPower(0);     //Stop the motors
        rightDrive.setPower(0);
    }
}
