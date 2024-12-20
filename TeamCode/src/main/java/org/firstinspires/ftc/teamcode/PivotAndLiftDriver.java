package org.firstinspires.ftc.teamcode;

// imports
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="PivotAndLift_Driver", group="Iterative OpMode")
public class PivotAndLiftDriver extends OpMode{

    ////////// TIME OBJECTS //////////
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime deltaTime = new ElapsedTime();

    ////////// ELECTRONICS REFERENCES //////////
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor pivotMotor = null; // reference to the motor that controls the lift's pivot
    private DcMotor liftMotor = null; // reference to the motor that controls the lift
    private Servo pivotServo = null;
    private Servo gripServo = null;

    ////////// Movement Constants //////////
    public static final double MIN_DEGREE = -95;
    public static final double MAX_DEGREE = 10;
    public static final double CUTOFF_PROPORTION = 0.05;
    public static final double PIVOT_SPEED_MULT = 0.8;

    public static final int COUNT_PER_REV = 28;
    public static final double GEAR_REDUCTION = 60*125/15;
    public static final double DEGREE_ADJUST = 0.78;

    public static final double COUNT_PER_DEGREE = DEGREE_ADJUST * COUNT_PER_REV * GEAR_REDUCTION / 360;
    public static final double MIN_COUNT = (MIN_DEGREE*COUNT_PER_DEGREE);
    public static final double MAX_COUNT = (MAX_DEGREE*COUNT_PER_DEGREE);
    public static final double CUTOFF_COUNT = CUTOFF_PROPORTION*(MAX_COUNT-MIN_COUNT);


    public static final double DRIVE_MOTOR_SPEED_MIN = 0.1;
    public static final double DRIVE_MOTOR_SPEED_MAX = 1.0;
    public static final int DRIVE_MOTOR_SPEED_LEVELS = 4;
    public static final double BUMPER_DEBOUNCE_TIME  = 10.0; // Minimum number of milliseconds between bumper presses

    // The calculated speed difference between each speed level
    public static final double DRIVE_MOTOR_SPEED_DIFF = DRIVE_MOTOR_SPEED_LEVELS == 1 ?  0 : (DRIVE_MOTOR_SPEED_MAX - DRIVE_MOTOR_SPEED_MIN) / (DRIVE_MOTOR_SPEED_LEVELS - 1);

    // DRIVE SPEED & BUMPER VARIABLES
    int currentSpeedLevel = 0; // Range is [0, MOTOR_SPEED_LEVELS)
    double bumperDecrementCooldown = 0.0;
    double bumperIncrementCooldown = 0.0;

    ////////// PIVOT MOTOR VARIABLES //////////
    private float targetPosition = 0;
    private boolean override = false;

    ////////// LIFT MOTOR VARIABLES //////////
    private float liftTargetPosition = 0;
    private static final float LIFT_MIN_ROTATION = 0;
    private static final float LIFT_MAX_ROTATION = 3000;

    ////////// Gripper //////////
    private double pivotServoOffset  = 0;
    private double gripServoOffset = 0;

    public static final double MID_SERVO   =  0.5 ;
    public static final double SERVO_SPEED  = 0.002 ;



    @Override
    public void init() {
        // Tell the driver that initialization is starting.
        telemetry.addData("Status", "Initializing");

        // get electronic references
        pivotMotor = hardwareMap.get(DcMotor.class, "pivotTest");
        liftMotor = hardwareMap.get(DcMotor.class, "lift");
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        pivotServo = hardwareMap.get(Servo.class, "pivotServo");
        gripServo = hardwareMap.get(Servo.class, "gripServo");
        telemetry.addData("Status", "Acquired Electronic References");

        // Set the encoder on the pivotMotor to run correctly
        pivotMotor.setDirection(DcMotor.Direction.FORWARD);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Status", "Initialized Pivot Motor");

        // Set up lift motor to work correctly
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Status", "Initialized Lift Motor");

        // Set up left and right drive to work correctly
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Status", "Initialized Drive Motors");

        // Set up gripper servo
        pivotServo.setPosition(MID_SERVO);
        gripServo.setPosition(MID_SERVO);


        telemetry.addData("Status", "Initializing Finished");
    }

    @Override
    public void init_loop() {
        if(gamepad1.dpad_up){
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor.setDirection(DcMotor.Direction.FORWARD);
            liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            telemetry.addData("Status", "reset moter");
        }
        if(gamepad1.b){
            liftMotor.setTargetPosition(0);
            telemetry.addData("Status", "Moving to zero");
        }
    }

    @Override
    public void start() {
        // Set the elapsed time to 0.
        runtime.reset();
        deltaTime.reset();
        liftTargetPosition = liftMotor.getCurrentPosition();
    }

    @Override
    public void loop() {
        double dt = deltaTime.milliseconds();
        runtime.reset();

        // OVERRODE LOGIC
        if (gamepad1.dpad_left && gamepad1.b) {
            pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addData("Pivot", "Reset Motor Encoder");
        }

        if (gamepad1.dpad_left && gamepad1.y) {
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addData("Lift", "Reset Lift");
        }

        if (gamepad1.dpad_right && gamepad1.x) {
            override = true;
        } else if (gamepad1.start) {
            override = false;
        }

        ////////// PIVOT LOGIC //////////

        int pivotPosition = pivotMotor.getCurrentPosition();
        double pivotPower = 0;

        pivotPower += gamepad1.left_stick_y * PIVOT_SPEED_MULT;

        if (MAX_COUNT-pivotPosition < CUTOFF_COUNT && pivotPower > 0 && CUTOFF_PROPORTION > 0 && !override) {
            pivotPower *= (MAX_COUNT-pivotPosition)/((double)CUTOFF_COUNT);
        }

        if (pivotPosition-MIN_COUNT < CUTOFF_COUNT && pivotPower < 0 && CUTOFF_PROPORTION > 0 && !override) {
            pivotPower *= (pivotPosition-MIN_COUNT)/((double)CUTOFF_COUNT);
        }

        pivotMotor.setPower(Range.clip(pivotPower,-1,1) * .5);


        ////////// LIFT LOGIC //////////
        int currentPosition = liftMotor.getCurrentPosition();
        int liftPower = 0;
        if(currentPosition <= LIFT_MAX_ROTATION || override){
            liftPower += gamepad1.left_trigger;
        }
        if(currentPosition >= LIFT_MIN_ROTATION || override){
            liftPower -= gamepad1.right_trigger;
        }

        liftMotor.setPower(Range.clip(liftPower, -1, 1));

        ////////// DRIVE & BUMPER LOGIC //////////

        handleBumperPress(dt, gamepad2.left_bumper, gamepad2.right_bumper);

        double drive = -gamepad2.left_stick_y;
        double turn  =  gamepad2.right_stick_x;
        double leftPower = drive + turn;
        double rightPower = drive - turn;

        double speedMult = DRIVE_MOTOR_SPEED_MIN + (currentSpeedLevel * DRIVE_MOTOR_SPEED_DIFF);
        leftPower = Range.clip(leftPower, -1.0, 1.0) * speedMult;
        rightPower = Range.clip(rightPower, -1.0, 1.0) * speedMult;

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);



        ////////// GIPPER CONTROLLERS //////////

        if(gamepad1.left_bumper){
            gripServoOffset += SERVO_SPEED;
        } else if(gamepad1.right_bumper){
            gripServoOffset -= SERVO_SPEED;
        }
        gripServoOffset = Range.clip(gripServoOffset, -0.5, 0.5);
        gripServo.setPosition(MID_SERVO + gripServoOffset);

        if(gamepad1.dpad_up){
            pivotServoOffset += SERVO_SPEED;
        } else if(gamepad1.dpad_down){
            pivotServoOffset -= SERVO_SPEED;
        }
        pivotServoOffset = Range.clip(pivotServoOffset, -0.5, 0.5);
        pivotServo.setPosition(MID_SERVO + pivotServoOffset);

        ////////// TELEMETRY OUTPUT //////////
        telemetry.addData("Override: ", override);
        telemetry.addData("------------", "" );
        // Run Time
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Status", "Delta Time: " + deltaTime.toString());
        telemetry.addData("------------", "" );
        // Lift
        telemetry.addData("Lift", "Power: " + liftPower);
        telemetry.addData("Lift", "Position: " + currentPosition);
        telemetry.addData("------------", "" );
        // Pivot
        telemetry.addData("Pivot", "Power:: " + pivotPower);
        telemetry.addData("Pivot", "Position: " + pivotPosition);
        telemetry.addData("------------", "" );
        // Drive
        telemetry.addData("Drive Motors", "Left: " + leftPower);
        telemetry.addData("Drive Motors", "Right: " + rightPower);
        telemetry.addData("------------", "" );
        // Gripper
        telemetry.addData("Grip Servo Position", "Left: " + gripServo.getPosition());
        telemetry.addData("Grip Pivot Servo Position", "Right: " + pivotServo.getPosition());
        telemetry.addData("------------", "" );


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

}
