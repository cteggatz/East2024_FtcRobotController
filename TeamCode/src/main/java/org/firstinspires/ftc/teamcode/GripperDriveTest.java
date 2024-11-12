package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Basic: Grip OpMode", group="Iterative OpMode")
public class GripperDriveTest extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Servo pivotServo = null;
    private Servo gripServo = null;

    private double pivotOffset = 0;
    private double gripOffset = 0;

    public static final double MID_SERVO   =  0.5 ;
    public static final double SERVO_SPEED  = 0.002 ;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        pivotServo = hardwareMap.get(Servo.class, "pivotServo");
        gripServo = hardwareMap.get(Servo.class, "gripServo");

        pivotServo.setPosition(MID_SERVO);
        gripServo.setPosition(MID_SERVO);

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
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry

        if(gamepad1.left_bumper){
            gripOffset += SERVO_SPEED;
        } else if(gamepad1.right_bumper){
            gripOffset -= SERVO_SPEED;
        }
        gripOffset = Range.clip(gripOffset, -0.5, 0.5);
        gripServo.setPosition(MID_SERVO + gripOffset);

        if(gamepad1.dpad_up){
            pivotOffset += SERVO_SPEED;
        } else if(gamepad1.dpad_down){
            pivotOffset -= SERVO_SPEED;
        }
        pivotOffset = Range.clip(pivotOffset, -0.5, 0.5);
        pivotServo.setPosition(MID_SERVO + pivotOffset);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }
}
