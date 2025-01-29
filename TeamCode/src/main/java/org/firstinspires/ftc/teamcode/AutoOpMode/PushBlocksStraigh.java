package org.firstinspires.ftc.teamcode.AutoOpMode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Managers.MotorManager;
import org.firstinspires.ftc.teamcode.drive.NewTankDrive;

@Config
@Autonomous(group = "autoModes")
public class PushBlocksStraigh extends LinearOpMode {

    //// DRIVE CONSTANTS ////
    private static final double turn90 = 90 * 1.6;

    //// PIVOT CONSTANTS ////
    public static final double PIVOT_MIN_COUNT = -7000;
    public static final double PIVOT_MAX_COUNT = 0;
    public static final double PIVOT_CUTOFF_COUNT = 300;
    public static final double PIVOT_MAINTAIN_COUNT = 50;

    //// LIFT ////
    public static final double LIFT_MIN_COUNT = -4000;
    public static final double LIFT_MAX_COUNT = 0;
    public static final double LIFT_CUTOFF_COUNT = 400;
    public static final double LIFT_MAINTAIN_COUNT = 20;

    @Override
    public void runOpMode() throws InterruptedException {
        ////// INIT //////

        // road runner
        NewTankDrive drive = new NewTankDrive(hardwareMap);



        // our motors
        DcMotor pivotMotor = hardwareMap.get(DcMotor.class, "pivotTest");
        DcMotor liftMotor = hardwareMap.get(DcMotor.class, "lift");
        Servo armServo = hardwareMap.get(Servo.class, "pivotServo");
        Servo gripServo = hardwareMap.get(Servo.class, "gripServo");
        telemetry.addData("Status", "Acquired Electronic References");

        // INITIALIZE PIVOT MOTOR //
        pivotMotor.setDirection(DcMotor.Direction.FORWARD);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorManager pivotManager = new MotorManager(28)// information from https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-188-1-ratio-24mm-length-8mm-rex-shaft-30-rpm-3-3-5v-encoder/
                .UsingGearReduction((20/125)*(((((1+(46/17))) * (1+(46/17))) * (1+(46/17))) * (1+(46/17)))) // 25:125 gear plus motor gear reduction
                .UsingCounts()
                .Min(PIVOT_MIN_COUNT, PIVOT_CUTOFF_COUNT)
                .Max(PIVOT_MAX_COUNT, PIVOT_CUTOFF_COUNT);
        //.Maintain(PIVOT_MAINTAIN_COUNT);
        telemetry.addData("Status", "Initialized Pivot Motor");


        //MotorManagerMovement bottem = new MotorManagerMovement(pivotManager, pivotMotor, 0, 3, telemetry);

        //MotorManagerMovement bottem = new MotorManagerMovement(pivotManager, pivotMotor, 0, 3, telemetry);
        Trajectory moveToBlocks = drive.trajectoryBuilder(new Pose2d())
                .forward(50)
                .build();
        Trajectory pushBlocks = drive.trajectoryBuilder(new Pose2d())
                .forward(7)
                .build();

        Trajectory moveBackwards = drive.trajectoryBuilder(new Pose2d())
                .back(44)
                .build();
        Trajectory moveForwards = drive.trajectoryBuilder(new Pose2d())
                .forward(25)
                .build();

        Trajectory inchBackwards = drive.trajectoryBuilder(new Pose2d())
                .back(4)
                .build();

        Trajectory pushBlocksToSpawn = drive.trajectoryBuilder(new Pose2d())
                .forward(50)
                .build();



        // START OP MODE
        waitForStart();



        // move in front of the blocks
        drive.followTrajectory(moveToBlocks);

        // push the blocks together
        drive.turn(-Math.toRadians(turn90));
        drive.followTrajectory(pushBlocks);

        drive.turn(-Math.toRadians(turn90+15));
        drive.followTrajectory(pushBlocksToSpawn);

        drive.followTrajectory(moveBackwards);
        drive.turn(Math.toRadians(45*1.6));
        drive.followTrajectory(pushBlocksToSpawn);

    }


}
