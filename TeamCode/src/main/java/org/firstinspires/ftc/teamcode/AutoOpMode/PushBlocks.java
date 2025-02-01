package org.firstinspires.ftc.teamcode.AutoOpMode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Managers.MotorManager;
import org.firstinspires.ftc.teamcode.drive.NewTankDrive;

@Config
@Autonomous(group = "autoModes")
public class PushBlocks extends LinearOpMode {

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
                .UsingGearIncrease(1062)// manually tuned instead of calculated
                .UsingCounts()
                .Min(PIVOT_MIN_COUNT, PIVOT_CUTOFF_COUNT)
                .Max(PIVOT_MAX_COUNT, PIVOT_CUTOFF_COUNT);
        //.Maintain(PIVOT_MAINTAIN_COUNT);
        telemetry.addData("Status", "Initialized Pivot Motor");

        // START OP MODE
        waitForStart();

        moveStraight(drive,50);
        drive.turn(-Math.toRadians(90 * 1.4));
        moveStraight(drive,7);
        drive.turn(-Math.toRadians(90 * 1.4));
        moveStraight(drive,35);
        moveStraight(drive,-45);
        drive.turn(-Math.toRadians(-15*3));
        moveStraight(drive,60);
    }

    private void moveStraight(NewTankDrive drive, double distance) {
        TrajectoryBuilder trajectory = drive.trajectoryBuilder(new Pose2d());
        if (distance > 0) trajectory.forward(distance);
        else if (distance < 0) trajectory.back(-distance);
        drive.followTrajectory(trajectory.build());
    }
}
