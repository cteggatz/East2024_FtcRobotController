package org.firstinspires.ftc.teamcode.AutoOpMode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.AutoMoves.MotorManagerMovement;
import org.firstinspires.ftc.teamcode.Managers.MotorManager;
import org.firstinspires.ftc.teamcode.drive.NewTankDrive;

@Config
@Autonomous(group = "autoModes")
public class TouchBar extends LinearOpMode {

    ////////// ELECTRONICS REFERENCES //////////
    private DcMotor pivotMotor = null; // reference to the motor that controls the lift's pivot
    private DcMotor liftMotor = null; // reference to the motor that controls the lift
    private Servo armServo = null;
    private Servo gripServo = null;

    //// PIVOT CONSTANTS ////
    private MotorManager pivotManager;
    public static final double PIVOT_MIN_COUNT = -7000;
    public static final double PIVOT_MAX_COUNT = -20;
    public static final double PIVOT_EDGE_COUNT = 300;
    public static final double PIVOT_ERROR_COUNT = 40;

    //// LIFT ////
    private MotorManager liftManager;
    public static final double LIFT_MIN_COUNT = -4400;
    public static final double LIFT_MIN_COUNT_LOW = -3100;
    public static final double LIFT_MIN_LOW_PROPORTION = LIFT_MIN_COUNT_LOW/LIFT_MIN_COUNT;
    public static final double LIFT_MAX_COUNT = -20;
    public static final double LIFT_EDGE_COUNT = 300;
    public static final double LIFT_MAINTAIN_COUNT = 80;

    @Override
    public void runOpMode() throws InterruptedException {
        ////// INIT //////

        // road runner
        NewTankDrive drive = new NewTankDrive(hardwareMap);

        // Tell the driver that initialization is starting.
        telemetry.addData("Status", "Initializing");

        // get electronic references
        pivotMotor = hardwareMap.get(DcMotor.class, "pivotTest");
        liftMotor = hardwareMap.get(DcMotor.class, "lift");
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
                .Max(PIVOT_MAX_COUNT, PIVOT_EDGE_COUNT)
                .AutoError(PIVOT_ERROR_COUNT);
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

        pivotManager.SetTargetRotation(-6850);
        liftManager.SetTargetRotation(0);
        armServo.setPosition(.143);
        while (!isStarted()) {
            pivotManager.UpdateRotation(pivotMotor.getCurrentPosition());
            pivotMotor.setPower(pivotManager.GetFinalPower());

            liftManager.UpdateRotation(liftMotor.getCurrentPosition());
            regulateLiftMin();
            liftMotor.setPower(liftManager.GetFinalPower());
        }

        moveStraight(drive,60);
        drive.turn(-Math.toRadians(90 * 1.4));
        moveStraight(drive,20);

        pivotManager.SetTargetRotation(-3900);
        liftManager.SetTargetRotation(-1600);
        while (!isStopRequested()) {
            pivotManager.UpdateRotation(pivotMotor.getCurrentPosition());
            pivotMotor.setPower(pivotManager.GetFinalPower());

            liftManager.UpdateRotation(liftMotor.getCurrentPosition());
            regulateLiftMin();
            liftMotor.setPower(liftManager.GetFinalPower());
        }
    }

    private void moveStraight(NewTankDrive drive, double distance) {
        TrajectoryBuilder trajectory = drive.trajectoryBuilder(new Pose2d());
        if (distance > 0) trajectory.forward(distance);
        else if (distance < 0) trajectory.back(-distance);
        drive.followTrajectory(trajectory.build());
    }

    private void regulateLiftMin() {
        pivotManager.UsingRadians();
        double rotMult = Math.cos(Math.toRadians(90) + pivotManager.GetRotation());
        double rotMult2 = rotMult <= 0 ? 1 : Math.min(1,LIFT_MIN_LOW_PROPORTION/rotMult);
        pivotManager.UsingCounts();
        liftManager.Min(LIFT_MIN_COUNT * rotMult2);
    }
}

/*{

    //// DRIVE CONSTANTS ////
    private static final double turn90 = 90 * 1.6;
    private static final double angleMult = -1.6*Math.PI/180;

    //// PIVOT CONSTANTS ////
    public static final double PIVOT_MIN_COUNT = -7000;
    public static final double PIVOT_MAX_COUNT = -20;
    public static final double PIVOT_EDGE_COUNT = 300;

    //// LIFT ////
    public static final double LIFT_MIN_COUNT = -4400;
    public static final double LIFT_MAX_COUNT = -20;
    public static final double LIFT_EDGE_COUNT = 300;
    public static final double LIFT_MAINTAIN_COUNT = 80;

    @Override
    public void runOpMode() throws InterruptedException {
        ////// INIT //////

        // road runner
        NewTankDrive drive = new NewTankDrive(hardwareMap);
        Trajectory trajectoryForward = drive.trajectoryBuilder(new Pose2d())
                .forward(60)
                .build();
        Trajectory trajectoryBackwards = drive.trajectoryBuilder(new Pose2d())
                .forward(20)
                .build();


        DcMotor pivotMotor = hardwareMap.get(DcMotor.class, "pivotTest");
        DcMotor liftMotor = hardwareMap.get(DcMotor.class, "lift");
        Servo armServo = hardwareMap.get(Servo.class, "pivotServo");
        Servo gripServo = hardwareMap.get(Servo.class, "gripServo");
        telemetry.addData("Status", "Acquired Electronic References");

        // INITIALIZE PIVOT MOTOR //
        pivotMotor.setDirection(DcMotor.Direction.FORWARD);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorManager pivotManager = new MotorManager(28)// information from https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-188-1-ratio-24mm-length-8mm-rex-shaft-30-rpm-3-3-5v-encoder/
                .UsingGearIncrease(1062)// manually tuned instead of calculated
                .UsingCounts()
                .Min(PIVOT_MIN_COUNT, PIVOT_EDGE_COUNT)
                .Max(PIVOT_MAX_COUNT, PIVOT_EDGE_COUNT)
                .AutoError(2);
        //.Maintain(PIVOT_MAINTAIN_COUNT);
        telemetry.addData("Status", "Initialized Pivot Motor");

        // Set up lift motor to work correctly
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        //liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorManager liftManager = new MotorManager(28)// information from https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-26-9-1-ratio-24mm-length-8mm-rex-shaft-223-rpm-3-3-5v-encoder/
                .UsingCounts()
                .Min(LIFT_MIN_COUNT, LIFT_EDGE_COUNT)
                .Max(LIFT_MAX_COUNT, LIFT_EDGE_COUNT)
                .Maintain(LIFT_MAINTAIN_COUNT);


        //MotorManagerMovement bottem = new MotorManagerMovement(pivotManager, pivotMotor, 0, 3, telemetry);

        // START OP MODE
        waitForStart();


        drive.followTrajectory(trajectoryForward);
        drive.turn(Math.toRadians(-90*1.4));

        // move to the mid point
        pivotManager.SetTargetRotation(pivotManager.FromProportionalRotation(.45));
        liftManager.SetTargetRotation(liftManager.FromProportionalRotation(.65));
        while (!pivotManager.IsNearTargetRotation() || liftManager.IsNearTargetRotation()) {

        }

        liftManager;
        lif
        sleep(10*1000);
    }
}
*/