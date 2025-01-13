package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Disabled//@TeleOp(name="Test: Iterative OpMode", group="Iterative OpMode")
public class TestOpMode extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorTest = null; // Local reference to the physical motor.
    private Servo   servoTest   = null; //Local reference to the physical motor.

    double servoOffset = 0; // Refers to the rotation of the servo.

    public static final double MID_SERVO   =  0.5 ;
    public static final double SERVO_SPEED  = 0.02 ;

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
        motorTest = hardwareMap.get(DcMotor.class, "motorTest");
        servoTest = hardwareMap.get(Servo.class, "servoTest");

        // Initialize the motor and servo to default settings.
        motorTest.setDirection(DcMotor.Direction.FORWARD);
        servoTest.setPosition(MID_SERVO);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
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
        // Set the elapsed time to 0.
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // Set motor power to y-axis of left stick.
        double drive = gamepad1.left_stick_y;
        double motorPower = Range.clip(drive, -1.0, 1.0) ;

        // Adjust servo rotation based on left and right bumpers.
        if (gamepad1.right_bumper)
            servoOffset += SERVO_SPEED;
        if (gamepad1.left_bumper)
            servoOffset -= SERVO_SPEED;
        servoOffset = Range.clip(servoOffset, -0.5, 0.5);

        // Sends motor and servo values to the hardware.
        motorTest.setPower(motorPower);
        servoTest.setPosition(MID_SERVO + servoOffset);

        // Tell the driver the current status.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motor", "Power = (%.2f)", motorPower);
        telemetry.addData("Servo", "Offset = %.2f", servoOffset);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }
}