package org.firstinspires.ftc.teamcode.AutoOpMode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AutoMoves.MotorManagerMovement;
import org.firstinspires.ftc.teamcode.Managers.MotorManager;
import org.firstinspires.ftc.teamcode.drive.NewTankDrive;

@Config
@Autonomous(group = "autoModes")
public class TestRoadRunner extends LinearOpMode {

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
                .UsingGearReduction((((1+(46/11))) * (1+(46/11))))
                .UsingCounts()
                .Min(LIFT_MIN_COUNT, LIFT_EDGE_COUNT)
                .Max(LIFT_MAX_COUNT, LIFT_EDGE_COUNT)
                .Maintain(LIFT_MAINTAIN_COUNT);


        //MotorManagerMovement bottem = new MotorManagerMovement(pivotManager, pivotMotor, 0, 3, telemetry);

        // START OP MODE
        waitForStart();


        drive.followTrajectory(trajectoryForward);
        drive.turn(-Math.toRadians(turn90+5));
        //drive.followTrajectory(trajectoryBackward);

        // move to the mid point
        MotorManagerMovement mid = new MotorManagerMovement(pivotManager, pivotMotor, 0.45, this);
        for(MotorManagerMovement move: mid){
            telemetry.addData("Stuff", "things");
            telemetry.update();
        }

        MotorManagerMovement extend = new MotorManagerMovement(liftManager, liftMotor, .65, this);
        for(MotorManagerMovement move: extend){
            telemetry.addData("Stuff", "things");
        }

        drive.followTrajectory(trajectoryBackwards);

        pivotMotor.setTargetPosition(0);
        sleep(10*1000);
    }
}
