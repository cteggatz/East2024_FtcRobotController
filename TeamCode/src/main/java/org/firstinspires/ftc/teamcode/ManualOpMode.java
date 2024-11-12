package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Manual: Basic OpMode", group="Iterative OpMode")
public class ManualOpMode extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    int currentSpeedLevel = 0; // Range is [0, MOTOR_SPEED_LEVELS)

    double bumperDecrementCooldown = 0.0;
    double bumperIncrementCooldown = 0.0;

    public static final double DRIVE_MOTOR_SPEED_MIN = 0.4;
    public static final double DRIVE_MOTOR_SPEED_MAX = 1.0;
    public static final int DRIVE_MOTOR_SPEED_LEVELS = 3;

    public static final double BUMPER_DEBOUNCE_TIME  = 10.0; // Minimum number of milliseconds between bumper presses

    // The calculated speed difference between each speed level
    public static final double DRIVE_MOTOR_SPEED_DIFF = DRIVE_MOTOR_SPEED_LEVELS == 1 ?  0 : (DRIVE_MOTOR_SPEED_MAX - DRIVE_MOTOR_SPEED_MIN) / (DRIVE_MOTOR_SPEED_LEVELS - 1);

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

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
        double deltaTime = runtime.milliseconds();
        runtime.reset();

        // Change speed depending on left and right bumper presses.
        handleBumperPress(deltaTime, gamepad1.left_bumper, gamepad1.right_bumper);

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_y;
        setMotorSpeed(drive + turn, drive - turn);

        // Show the elapsed game time.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

    /**
     * If either of the bumpers are being pressed, update currentSpeedLevel and handle button press debounce.
     * @param deltaTime The time since the function was last called, in milliseconds.
     * @param decrementButtonPressed Whether the button to decrement currentSpeedLevel is being pressed.
     * @param incrementButtonPressed Whether the button to increment currentSpeedLevel is being pressed.
     */
    private void handleBumperPress(double deltaTime, boolean decrementButtonPressed, boolean incrementButtonPressed) {
        // Decrease debounce cooldown by time passed.
        bumperDecrementCooldown -= deltaTime;
        if (bumperDecrementCooldown < 0.0) bumperDecrementCooldown = 0.0;

        if (decrementButtonPressed) {
            // If not on debounce cooldown, try to decrement speed level.
            if (bumperDecrementCooldown == 0.0) {
                currentSpeedLevel--;
                if (currentSpeedLevel < 0) currentSpeedLevel = 0;
            }

            // Reset debounce cooldown.
            bumperDecrementCooldown = BUMPER_DEBOUNCE_TIME;
        }

        // Decrease debounce cooldown by time passed.
        bumperIncrementCooldown -= deltaTime;
        if (bumperIncrementCooldown < 0.0) bumperIncrementCooldown = 0.0;

        if (incrementButtonPressed) {
            // If not on debounce cooldown, try to increment speed level.
            if (bumperIncrementCooldown == 0.0) {
                currentSpeedLevel++;
                if (currentSpeedLevel > DRIVE_MOTOR_SPEED_LEVELS - 1) currentSpeedLevel = DRIVE_MOTOR_SPEED_LEVELS - 1;
            }

            // Reset debounce cooldown.
            bumperIncrementCooldown = BUMPER_DEBOUNCE_TIME;
        }
    }

    private void setMotorSpeed(double leftPower, double rightPower) {
        double speedMult = DRIVE_MOTOR_SPEED_MIN + (currentSpeedLevel * DRIVE_MOTOR_SPEED_DIFF);
        leftPower = Range.clip(leftPower, -1.0, 1.0) * speedMult;
        rightPower = Range.clip(rightPower, -1.0, 1.0) * speedMult;

        // Send calculated power to wheels and display status
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }
}