package org.firstinspires.ftc.teamcode.Macros;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.AutoMoves.AutoModeMovements;
import org.firstinspires.ftc.teamcode.AutoMoves.ExtendLift;
import org.firstinspires.ftc.teamcode.AutoMoves.GripPivot;
import org.firstinspires.ftc.teamcode.AutoMoves.Gripper;
import org.firstinspires.ftc.teamcode.AutoMoves.MultiMove;
import org.firstinspires.ftc.teamcode.AutoMoves.Pause;
import org.firstinspires.ftc.teamcode.AutoMoves.PivotArm;
import org.firstinspires.ftc.teamcode.AutoMoves.QueueMove;
import org.firstinspires.ftc.teamcode.MotorData;

import java.util.Queue;

public class ResetArm extends QueueMove {
    public ResetArm(Servo gripServo, Servo pivotServo, DcMotor liftMotor, DcMotor pivotMotor) {
        super(
                new AutoModeMovements[]{
                        // move upwards and pull in lift
                        new PivotArm(pivotMotor, (int)MotorData.MIN_COUNT),
                        new Pause(250),
                        new ExtendLift(liftMotor, 0),
                        new Pause(250),
                        // move to neutral
                        new MultiMove(new AutoModeMovements[]{
                                new GripPivot(pivotServo,.6),
                                new Gripper(gripServo,.9),
                                new PivotArm(pivotMotor, 0),
                        })
                }
        );
    }
}
