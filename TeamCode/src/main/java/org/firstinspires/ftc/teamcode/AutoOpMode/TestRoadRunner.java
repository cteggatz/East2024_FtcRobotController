package org.firstinspires.ftc.teamcode.AutoOpMode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.NewTankDrive;

@Config
@Autonomous(group = "drive")
public class TestRoadRunner extends LinearOpMode {

    private static final double turn90 = 90 * 1.6;

    @Override
    public void runOpMode() throws InterruptedException {
        NewTankDrive drive = new NewTankDrive(hardwareMap);

        Trajectory trajectoryForward = drive.trajectoryBuilder(new Pose2d())
                .forward(70)
                .build();
        Trajectory trajectoryBackwards = drive.trajectoryBuilder(new Pose2d())
                .back(20)
                .build();


        waitForStart();

        drive.followTrajectory(trajectoryForward);
        drive.turn(Math.toRadians(turn90));
        drive.followTrajectory(trajectoryBackwards);

        //drive.followTrajectory(trajectoryBackward);
    }
}
