package org.firstinspires.ftc.teamcode.Macros;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.AutoMoves.AutoModeMovements;
import org.firstinspires.ftc.teamcode.AutoMoves.GripPivot;
import org.firstinspires.ftc.teamcode.AutoMoves.Gripper;
import org.firstinspires.ftc.teamcode.AutoMoves.MultiMove;
import org.firstinspires.ftc.teamcode.AutoMoves.Pause;
import org.firstinspires.ftc.teamcode.AutoMoves.QueueMove;

public class PickUp extends QueueMove {

    public PickUp(Servo gripServo, Servo pivotServo, DcMotor liftMotor, DcMotor pivotMotor){
        super(new AutoModeMovements[]{
                // Open the claw and move it down
                new MultiMove( new AutoModeMovements[]{
                        new GripPivot(pivotServo, .5),
                        new Gripper(gripServo, .4),
                }),
                new Pause(500),
                new MultiMove( new AutoModeMovements[] {
                        new Gripper(gripServo, .4),
                        new GripPivot(pivotServo, .9),
                }),
                new Pause(500),
                new Gripper(gripServo, 1),
                new Pause(500),
                new MultiMove( new AutoModeMovements[]{
                        new GripPivot(pivotServo, .1),
                        new Gripper(gripServo, 1)
                }),
        });
    }
}
