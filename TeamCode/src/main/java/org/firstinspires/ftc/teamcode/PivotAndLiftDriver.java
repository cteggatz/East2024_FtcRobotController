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
    public static final int COUNT_PER_REV = 28;
    public static final int GEAR_REDUCTION = 60*125/15;
    public static final int COUNT_PER_DEGREE = COUNT_PER_REV * GEAR_REDUCTION / 360;
    public static final int MIN_DEGREE = -20;
    public static final int MAX_DEGREE = 90;

    public static final double MOVE_SPEED = .05/1000;

    ////////// PIVOT MOTOR VARIABLES //////////
    private float targetPosition = 0;

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

        ////////// PIVOT LOGIC //////////

        int pivotPosition = pivotMotor.getCurrentPosition();
        float pivotPower = 0;

        if(gamepad1.right_bumper){
            pivotPower = 0.5f;
        } else if(gamepad1.left_bumper){
            pivotPower = -0.5f;
        }
        pivotMotor.setPower(pivotPower);


        ////////// LIFT LOGIC //////////
        int currentPosition = liftMotor.getCurrentPosition();
        int liftPower = 0;
        if(gamepad1.left_trigger == 1 && currentPosition < 3000){
            liftPower = 1;
        } else if(gamepad1.right_trigger == 1 && currentPosition > 0){
            liftPower = -1;
        }
        liftMotor.setPower(liftPower);

        ////////// TELEMETRY OUTPUT //////////
        // Run Time
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Status", "Delta Time: " + deltaTime.toString());
        telemetry.addData("------------", "" );
        // Lift
        telemetry.addData("Lift", "Target Position: " + liftPower);
        telemetry.addData("Lift", "CurrentPosition: " + currentPosition);
        telemetry.addData("------------", "" );
        // Pivot
        telemetry.addData("Pivot", "Target Position: " + pivotPower);
        telemetry.addData("Pivot", "CurrentPosition: " + pivotPosition);
        telemetry.addData("------------", "" );


    }
}