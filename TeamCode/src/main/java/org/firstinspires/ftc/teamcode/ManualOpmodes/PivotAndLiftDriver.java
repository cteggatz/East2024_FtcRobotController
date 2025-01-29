package org.firstinspires.ftc.teamcode.ManualOpmodes;

// imports
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Managers.MotorManager;

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
    private Servo armServo = null;
    private Servo gripServo = null;

    private boolean override = false;

    ////////// Movement Constants //////////
    public static final double DRIVE_MOTOR_SPEED_MIN = 0.1;
    public static final double DRIVE_MOTOR_SPEED_MAX = 1.0;
    public static final double FINE_SPEED = 0.3;
    public static final int DRIVE_MOTOR_SPEED_LEVELS = 4;
    public static final double BUMPER_DEBOUNCE_TIME  = 0.100; // Minimum number of seconds between bumper presses

    // The calculated speed difference between each speed level
    public static final double DRIVE_MOTOR_SPEED_DIFF = DRIVE_MOTOR_SPEED_LEVELS == 1 ?  0 : (DRIVE_MOTOR_SPEED_MAX - DRIVE_MOTOR_SPEED_MIN) / (DRIVE_MOTOR_SPEED_LEVELS - 1);

    // DRIVE SPEED & BUMPER VARIABLES
    int currentSpeedLevel = 0; // Range is [0, MOTOR_SPEED_LEVELS)
    double bumperDecrementCooldown = 0.0;
    double bumperIncrementCooldown = 0.0;

    ////////// PIVOT MOTOR VARIABLES //////////
    private MotorManager pivotManager;
    public static final double PIVOT_MIN_COUNT = -7000;
    public static final double PIVOT_MAX_COUNT = -20;
    public static final double PIVOT_EDGE_COUNT = 300;

    ////////// LIFT MOTOR VARIABLES //////////
    private MotorManager liftManager;
    public static final double LIFT_MIN_COUNT = -4400;
    public static final double LIFT_MIN_COUNT_LOW = -3100;
    public static final double LIFT_MIN_LOW_PROPORTION = LIFT_MIN_COUNT_LOW/LIFT_MIN_COUNT;
    public static final double LIFT_MAX_COUNT = -20;
    public static final double LIFT_EDGE_COUNT = 300;
    public static final double LIFT_MAINTAIN_COUNT = 80;


    ////////// GRIPPER MOTOR VARIABLES //////////
    private double armPosition = 0;
    private double gripPosition = 0;
    private boolean hasGripMoved = false;
    private boolean hasArmMoved = false;

    public static final double ARM_SPEED  = 0.3;
    public static final double GRIP_SPEED  = 0.5;



    @Override
    public void init() {
        // Tell the driver that initialization is starting.
        telemetry.addData("Status", "Initializing");

        // get electronic references
        pivotMotor = hardwareMap.get(DcMotor.class, "pivotTest");
        liftMotor = hardwareMap.get(DcMotor.class, "lift");
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        armServo = hardwareMap.get(Servo.class, "pivotServo");
        gripServo = hardwareMap.get(Servo.class, "gripServo");
        telemetry.addData("Status", "Acquired Electronic References");

        // Set the encoder on the pivotMotor to run correctly
        pivotMotor.setDirection(DcMotor.Direction.FORWARD);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotManager = new MotorManager(28)// information from https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-188-1-ratio-24mm-length-8mm-rex-shaft-30-rpm-3-3-5v-encoder/
                .UsingGearIncrease(1062)// manually tuned instead of calculated
                .UsingCounts()
                .Min(PIVOT_MIN_COUNT, PIVOT_EDGE_COUNT)
                .Max(PIVOT_MAX_COUNT, PIVOT_EDGE_COUNT);
        telemetry.addData("Status", "Initialized Pivot Motor");

        // Set up lift motor to work correctly
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        //liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftManager = new MotorManager(28)// information from https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-26-9-1-ratio-24mm-length-8mm-rex-shaft-223-rpm-3-3-5v-encoder/
                .UsingCounts()
                .Min(LIFT_MIN_COUNT, LIFT_EDGE_COUNT)
                .Max(LIFT_MAX_COUNT, LIFT_EDGE_COUNT)
                .Maintain(LIFT_MAINTAIN_COUNT);
        telemetry.addData("Status", "Initialized Lift Motor");

        // Set up left and right drive to work correctly
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Status", "Initialized Drive Motors");

        // Set up gripper servo
        armPosition = armServo.getPosition();
        gripPosition = gripServo.getPosition();
        telemetry.addData("Status", armPosition);

        telemetry.addData("Status", "Initializing Finished");
    }

    @Override
    public void init_loop() {
       /* if(gamepad1.dpad_up){
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor.setDirection(DcMotor.Direction.FORWARD);
            liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            telemetry.addData("Status", "reset moter");
        }
        if(gamepad1.b){
            liftMotor.setTargetPosition(0);
            telemetry.addData("Status", "Moving to zero");
        }*/
    }

    @Override
    public void start() {
        // Set the elapsed time to 0.
        runtime.reset();
        deltaTime.reset();
        //armPosition = 0.6;//armServo.getPosition()
        //gripPosition = gripServo.getPosition();

    }

    @Override
    public void loop() {
        double dt = deltaTime.seconds();
        deltaTime.reset();

        // OVERRODE LOGIC
        if (gamepad1.back && gamepad1.x) {
            pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addData("Pivot", "Reset Motor Encoder");
        }

        if (gamepad1.back && gamepad1.b) {
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addData("Lift", "Reset Lift");
        }

        if (gamepad1.back && gamepad1.y) {
            override = true;
        } else if (gamepad1.back && gamepad1.a) {
            override = false;
        }

        ////////// PIVOT LOGIC //////////
        pivotManager.UpdateRotation(pivotMotor.getCurrentPosition());
        pivotManager.SetTargetPower(-improveInput(gamepad1.left_stick_y));
        double pivotPosition = pivotManager.GetRotation();
        double pivotPower = pivotManager.GetFinalPower(override);
        pivotMotor.setPower(pivotPower);

        ////////// LIFT LOGIC //////////
        liftManager.UpdateRotation(liftMotor.getCurrentPosition());
        liftManager.SetTargetPower(improveInput(gamepad1.left_trigger)-improveInput(gamepad1.right_trigger));

        regulateLiftMin();

        double liftPosition = liftManager.GetRotation();
        double liftPower = liftManager.GetFinalPower(override);

        liftMotor.setPower(liftPower);

        ////////// DRIVE & BUMPER LOGIC //////////

        handleBumperPress(dt, gamepad2.left_bumper, gamepad2.right_bumper);

        double speedMult = DRIVE_MOTOR_SPEED_MIN + (currentSpeedLevel * DRIVE_MOTOR_SPEED_DIFF);
        double drive = -improveInput(gamepad2.left_stick_y) * speedMult;
        double turn  =  improveInput(gamepad2.right_stick_x) * speedMult;
        if (gamepad1.dpad_up) drive += FINE_SPEED ;
        if (gamepad1.dpad_down) drive -= FINE_SPEED;
        if (gamepad1.dpad_right) turn += FINE_SPEED;
        if (gamepad1.dpad_left) turn -= FINE_SPEED ;
        double leftPower = drive + turn;
        double rightPower = drive - turn;

        leftPower = Range.clip(leftPower, -1.0, 1.0);
        rightPower = Range.clip(rightPower, -1.0, 1.0);

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);



        ////////// GRIPPER CONTROLLERS //////////
        // gripper

        if (!hasArmMoved) {
            armPosition = armServo.getPosition();
            if (gamepad1.right_stick_y != 0)
                hasArmMoved = true;
        }

        if (!hasGripMoved) {
            gripPosition = gripServo.getPosition();
            if (gamepad1.left_bumper || gamepad1.right_bumper)
                hasGripMoved = true;
        }

        if(gamepad1.left_bumper)
            gripPosition += GRIP_SPEED * dt;


        if(gamepad1.right_bumper)
            gripPosition -= GRIP_SPEED * dt;


        gripPosition = Range.clip(gripPosition, 0.83, 0.9);
        gripServo.setPosition(gripPosition);

        armPosition += -improveInput(gamepad1.right_stick_y) * ARM_SPEED * dt;
        armPosition = Range.clip(armPosition, 0.1, 1);
        armServo.setPosition(armPosition);

        ////////// TELEMETRY OUTPUT //////////
        telemetry.addData("Override: ", override);
        telemetry.addData("------------", "" );
        // Run Time
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Status", "Delta Time: " + deltaTime.toString());
        telemetry.addData("------------", "" );
        // Lift
        telemetry.addData("Lift", "Power: " + liftPower);
        telemetry.addData("Lift", "Position: " + liftPosition);
        telemetry.addData("------------", "" );
        // Pivot
        telemetry.addData("Pivot", "Power:: " + pivotPower);
        telemetry.addData("Pivot", "Position: " + pivotPosition);
        telemetry.addData("------------", "" );
        // Drive
        telemetry.addData("Drive Motors", "Left: " + leftPower);
        telemetry.addData("Drive Motors", "Right: " + rightPower);
        telemetry.addData("Drive Motors", "Speed: " + speedMult);
        telemetry.addData("------------", "" );
        // Gripper
        telemetry.addData("Grip Servo Position", "Left: " + gripServo.getPosition());
        telemetry.addData("Grip Pivot Servo Position", "Right: " + armServo.getPosition());
        telemetry.addData("------------", "" );


    }

    private double improveInput(double value) {
        return Math.signum(value) * (0.1 + 0.9*(value * value));
    }


    /**
     * If either of the bumpers are being pressed, update currentSpeedLevel and handle button press debounce.
     * @param deltaTime The time since the function was last called, in seconds.
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

    private void regulateLiftMin() {
        pivotManager.UsingRadians();
        double rotMult = Math.cos(Math.toRadians(90) + pivotManager.GetRotation());
        double rotMult2 = rotMult <= 0 ? 1 : Math.min(1,LIFT_MIN_LOW_PROPORTION/rotMult);
        pivotManager.UsingCounts();
        liftManager.Min(LIFT_MIN_COUNT * rotMult2);
    }
}
