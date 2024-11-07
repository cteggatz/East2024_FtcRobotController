package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Test: Pivot Motor", group="Iterative OpMode")
public class MotorTestOpMode extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    double targetPosition = 0;
    double moveChange = 0;
    //private DcMotor motorTest = null; // Local reference to the physical motor.
    private DcMotor pivotTest = null; //Local reference to the physical motor.

    double servoOffset = 0; // Refers to the rotation of the servo.

    public static final double MOVE_SPEED   =  0.5 ;

    public static final int COUNT_PER_REV = 28;
    public static final int GEAR_REDUCTION = 60*125/15;
    public static final int COUNT_PER_DEGREE = COUNT_PER_REV*GEAR_REDUCTION/360;
    public static final int MIN_DEGREE = -70;
    public static final int MAX_DEGREE = 10;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Tell the driver that initialization is starting.
        telemetry.addData("Status", "Initializing");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        //motorTest = hardwareMap.get(DcMotor.class, "motorTest");
        pivotTest = hardwareMap.get(DcMotor.class, "pivotTest");

        // Initialize the motor and servo to default settings.
        //motorTest.setDirection(DcMotor.Direction.FORWARD);
        pivotTest.setDirection(DcMotor.Direction.FORWARD);

        pivotTest.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotTest.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivotTest.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        pivotTest.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Run Mode", "Run_To_Position");

        // setting the target position
        this.targetPosition = pivotTest.getTargetPosition();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        if (gamepad1.dpad_left && gamepad1.b) {
            pivotTest.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            pivotTest.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            pivotTest.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry.addData("Status", "Reset Motor Encoder");
        }
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        // Set the elapsed time to 0.
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // Adjust servo rotation based on left and right bumpers.
        double deltaTime = runtime.milliseconds();
        runtime.reset();

        this.moveChange = 0;
        if(gamepad1.left_bumper){
            this.targetPosition += -.5 * deltaTime;
        } if (gamepad1.right_bumper) {
            this.targetPosition += .5 * deltaTime;
        }





        pivotTest.setTargetPosition((int)targetPosition);
        pivotTest.setPower(1.0f);

        // Tell the driver the current status.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        //telemetry.addData("Motor", "Power = (%.2f)", motorPower);
        telemetry.addData("Target Position", "Pos = (%7d)", (int)targetPosition);
        telemetry.addData("Current Position", "Pos = (%7d)", this.pivotTest.getCurrentPosition());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }
}