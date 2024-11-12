package org.firstinspires.ftc.teamcode;

// imports
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="PivotAndLift_Driver", group="Iterative OpMode")
public class PivotAndLiftDriver extends OpMode{

    ////////// TIME OBJECTS //////////
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime deltaTime = new ElapsedTime();

    ////////// ELECTRONICS REFERENCES //////////
    private DcMotor pivotMotor = null; // reference to the motor that controls the lift's pivot
    private DcMotor liftMotor = null; // reference to the motor that controls the lift

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

    ////////// PIVOT MOTOR VARIABLES //////////
    private float targetPosition = 0;
    private boolean override = false;

    ////////// LIFT MOTOR VARIABLES //////////
    private float liftTargetPosition = 0;
    private static final float LIFT_MIN_ROTATION = 0;
    private static final float LIFT_MAX_ROTATION = 3000;



    @Override
    public void init() {
        // Tell the driver that initialization is starting.
        telemetry.addData("Status", "Initializing");

        // get electronic references
        pivotMotor = hardwareMap.get(DcMotor.class, "pivotTest");
        liftMotor = hardwareMap.get(DcMotor.class, "lift");
        telemetry.addData("Status", "Got Electronic References");

        // Set the encoder on the pivotMotor to run correctly
        pivotMotor.setDirection(DcMotor.Direction.FORWARD);
        //pivotMotor.setTargetPosition(0);
        //pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Status", "Set Pivot Motor");

        // Set up lift motor to work correctly
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Status", "Set Lift Motor");

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {

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

        if (gamepad1.dpad_left && gamepad1.b) {
            pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addData("Pivot", "Reset Motor Encoder");
        }

        if (gamepad1.dpad_right && gamepad1.x) {
            override = true;
        } else if (gamepad1.start) {
            override = false;
        }

        ////////// PIVOT LOGIC //////////

        int pivotPosition = pivotMotor.getCurrentPosition();
        double pivotPower = 0;

        pivotPower += gamepad1.right_stick_y * PIVOT_SPEED_MULT;

        if (MAX_COUNT-pivotPosition < CUTOFF_COUNT && pivotPower > 0 && CUTOFF_PROPORTION > 0 && !override) {
            pivotPower *= (MAX_COUNT-pivotPosition)/((double)CUTOFF_COUNT);
        }

        if (pivotPosition-MIN_COUNT < CUTOFF_COUNT && pivotPower < 0 && CUTOFF_PROPORTION > 0 && !override) {
            pivotPower *= (pivotPosition-MIN_COUNT)/((double)CUTOFF_COUNT);
        }

        pivotMotor.setPower(Range.clip(pivotPower,-1,1));


        ////////// LIFT LOGIC //////////
        int currentPosition = liftMotor.getCurrentPosition();
        int liftPower = 0;
        if(gamepad1.left_trigger == 1 && (currentPosition < LIFT_MAX_ROTATION || !override)){
            liftPower += 1;
        }
        if(gamepad1.right_trigger == 1 && currentPosition > LIFT_MIN_ROTATION){
            liftPower -= 1;
        }

        liftMotor.setPower(liftPower);

        ////////// TELEMETRY OUTPUT //////////
        // Run Time
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Status", "Delta Time: " + deltaTime.toString());
        telemetry.addData("------------", "" );
        // Lift
        telemetry.addData("Lift", "Target Position: " + liftPower);
        telemetry.addData("Lift", "Current Position: " + currentPosition);
        telemetry.addData("------------", "" );
        // Pivot
        telemetry.addData("Pivot", "Target Position: " + pivotPower);
        telemetry.addData("Pivot", "Current Position: " + pivotPosition);
        telemetry.addData("------------", "" );


    }
}
