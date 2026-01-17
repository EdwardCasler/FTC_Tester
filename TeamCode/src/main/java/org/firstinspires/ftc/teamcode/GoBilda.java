

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;



@TeleOp(name = "QuickCode", group = "StarterBot")
//@Disabled
public class GoBilda extends OpMode {
    final double FEED_TIME_SECONDS = 0.20; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;

    //intake
    boolean intakeOn = false;     // Tracks whether intake is running
    boolean lastLB = false;       // Tracks previous state of left bumper
    boolean intakeReverse = false;   // Tracks whether intake direction is reversed
    boolean lastLT = false;          // Tracks previous state of left trigger

    /*
     * When we control our launcher motor, we are using encoders. These allow the control system
     * to read the current speed of the motor and apply more or less power to keep it at a constant
     * velocity. Here we are setting the target, and minimum velocity that the launcher should run
     * at. The minimum velocity is a threshold for determining when to fire.
     */
    final double LAUNCHER_TARGET_VELOCITY = 1125;
    final double LAUNCHER_MIN_VELOCITY = 1075;

    // Declare OpMode members.
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx launcher = null;
    private DcMotorEx intake = null;

    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    ElapsedTime feederTimer = new ElapsedTime();

    /*
     * TECH TIP: State Machines
     * We use a "state machine" to control our launcher motor and feeder servos in this program.
     * The first step of a state machine is creating an enum that captures the different "states"
     * that our code can be in.
     * The core advantage of a state machine is that it allows us to continue to loop through all
     * of our code while only running specific code when it's necessary. We can continuously check
     * what "State" our machine is in, run the associated code, and when we are done with that step
     * move on to the next state.
     * This enum is called the "LaunchState". It reflects the current condition of the shooter
     * motor and we move through the enum when the user asks our code to fire a shot.
     * It starts at idle, when the user requests a launch, we enter SPIN_UP where we get the
     * motor up to speed, once it meets a minimum speed then it starts and then ends the launch process.
     * We can use higher level code to cycle through these states. But this allows us to write
     * functions and autonomous routines in a way that avoids loops within loops, and "waits".
     */
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }

    private LaunchState launchState;

    // Setup a variable for each drive wheel to save power level for telemetry
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        launchState = LaunchState.IDLE;

        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step.
         */
        //Drive
        leftFrontDrive = hardwareMap.get(DcMotor.class, "fl");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "fr");
        leftBackDrive = hardwareMap.get(DcMotor.class, "bl");
        rightBackDrive = hardwareMap.get(DcMotor.class, "br");
        //launcher
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        //Feeders
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
        //intake
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make robot go forward. So adjust these two lines based on your first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90 Deg drives may require direction flips
         */
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        launcher.setDirection(DcMotor.Direction.REVERSE);

        /*
         * Here we set our launcher to the RUN_USING_ENCODER runmode.
         * If you notice that you have no control over the velocity of the motor, it just jumps
         * right to a number much higher than your set point, make sure that your encoders are plugged
         * into the port right beside the motor itself. And that the motors polarity is consistent
         * through any wiring.
         */
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
            //Hi
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);

        /*
         * set Feeders to an initial value to initialize the servo controller
         */
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));


        /*
         * Much like our drivetrain motors, we set the left feeder servo to reverse so that they
         * both work to feed the ball into the robot.
         */
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
        telemetry.addLine("=== Controller Button Map ===");
        telemetry.addData("Triangle (Y)", "Spin up launcher");
        telemetry.addData("Circle (B)", "Stop launcher");
        telemetry.addData("Cross (A)", "Reverse feeders");
        telemetry.addData("Square (X)", "Reverse launcher direction");
        telemetry.addData("Right Bumper (R1)", "Fire shot");
        telemetry.addData("Right Trigger (R2)", "Feed Rollers");

        telemetry.addData("Left Bumper (L1)", "Intake toggle: on/off");
        telemetry.addData("Left Trigger (L2)", "Intake direction reverse");
        telemetry.addData("Left Stick", "Drive forward/back & strafe");
        telemetry.addData("Right Stick X", "Rotate robot");
        telemetry.addLine("=============================");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        /*
         * Here we call a function called arcadeDrive. The arcadeDrive function takes the input from
         * the joysticks, and applies power to the left and right drive motor to move the robot
         * as requested by the driver. "arcade" refers to the control style we're using here.
         * Much like a classic arcade game, when you move the left joystick forward both motors
         * work to drive the robot forward, qh when you move the right joystick left and right
         * both motors work to rotate the robot. Combinations of these inputs can be used to create
         * more complex maneuvers.
         */
        mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        /*
         * Here we give the user control of the speed of the launcher motor without automatically
         * queuing a shot.
         */
        if (gamepad1.y) {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else if (gamepad1.b) { // stop flywheel
            launcher.setVelocity(STOP_SPEED);
        } else if(gamepad1.a){
            rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
            leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        } else if(gamepad1.x){
            launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        }




                // Detect a NEW press (button went from not pressed → pressed)
                if (gamepad1.left_bumper && !lastLB) {
                    intakeOn = !intakeOn;   // Toggle the intake state
                }

                // Apply motor power based on toggle state
                if (intakeOn) {
                    intake.setPower(1.0);   // Intake ON
                } else {
                    intake.setPower(0.0);   // Intake OFF
                }

                lastLB = gamepad1.left_bumper;  // Update for next loop

        // Manual feed roller control using R2
                if (gamepad1.right_trigger > 0.5) {
                    leftFeeder.setPower(1.0);
                    rightFeeder.setPower(1.0);
                } else {
                    leftFeeder.setPower(0.0);
                    rightFeeder.setPower(0.0);
                }


        // Read trigger state (pressed if > 0.5)
        boolean currentLT = gamepad1.left_trigger > 0.5;

        // Detect NEW press (edge detection)
                if (currentLT && !lastLT) {
                    intakeReverse = !intakeReverse;   // Toggle direction
                }

        // Apply direction
                if (intakeReverse) {
                    intake.setDirection(DcMotorSimple.Direction.REVERSE);
                } else {
                    intake.setDirection(DcMotorSimple.Direction.FORWARD);
                }

        // Update last state
                lastLT = currentLT;

        /*
         * Now we call our "Launch" function.
         */
        launch(gamepad1.rightBumperWasPressed());

        /*
         * Show the state and motor powers
         */
        telemetry.addData("State", launchState);
        telemetry.addData("motorSpeed", launcher.getVelocity());
        telemetry.addLine("=== Robot Status ===");

// Drivetrain (optional but helpful)
        telemetry.addData("LF Power", "%.2f", leftFrontPower);
        telemetry.addData("RF Power", "%.2f", rightFrontPower);
        telemetry.addData("LB Power", "%.2f", leftBackPower);
        telemetry.addData("RB Power", "%.2f", rightBackPower);

// Launcher
        telemetry.addLine("--- Launcher ---");
        telemetry.addData("State", launchState);
        telemetry.addData("Velocity", "%.1f", launcher.getVelocity());
        telemetry.addData("Target Vel", LAUNCHER_TARGET_VELOCITY);
        telemetry.addData("Min Vel", LAUNCHER_MIN_VELOCITY);

// Intake
        telemetry.addLine("--- Intake ---");
        telemetry.addData("Intake ON", intakeOn ? "YES" : "NO");
        telemetry.addData("Direction", intakeReverse ? "REVERSE" : "FORWARD");
        telemetry.addData("Power", "%.2f", intakeOn ? 1.0 : 0.0);

// Gamepad
        telemetry.addLine("--- Gamepad ---");
        telemetry.addData("L1 (Toggle)", gamepad1.left_bumper);
        telemetry.addData("L2 (Reverse Toggle)", "%.2f", gamepad1.left_trigger);
        //Feedrollers
        telemetry.addData("Feed Rollers", gamepad1.right_trigger > 0.5 ? "ON (R2)" : "OFF");

        telemetry.update();


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        telemetry.addLine("Program Stopped. Here are the logs so I know how the game went.");
        telemetry.addData("Launcher Velocity", launcher.getVelocity());
        telemetry.update();
    }

    void mecanumDrive(double forward, double strafe, double rotate){

        /* the denominator is the largest motor power (absolute value) or 1
         * This ensures all the powers maintain the same ratio,
         * but only if at least one is out of the range [-1, 1]
         */
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        leftFrontPower = (forward + strafe + rotate) / denominator;
        rightFrontPower = (forward - strafe - rotate) / denominator;
        leftBackPower = (forward - strafe + rotate) / denominator;
        rightBackPower = (forward + strafe - rotate) / denominator;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

    }

    void launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                    leftFeeder.setPower(STOP_SPEED);
                    rightFeeder.setPower(STOP_SPEED);
                }
                break;
        }
    }
}