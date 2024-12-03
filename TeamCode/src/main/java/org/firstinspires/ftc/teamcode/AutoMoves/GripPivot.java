package org.firstinspires.ftc.teamcode.AutoMoves;

import android.util.Pair;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MotorData;

public class GripPivot extends AutoModeMovements{
    private Servo gripperPivotServo;
    double targetPosition;
    boolean initialized = false;

    public GripPivot(Servo gripperServo, double targetPosition){
        this.gripperPivotServo = gripperServo;
        this.targetPosition = targetPosition * .9;
    }

    @Override
    public boolean isDone() {
        //telemetry.addData("isDone", "" + Math.abs(targetPosition - pivotMotor.getCurrentPosition()));
        return Math.abs(targetPosition - gripperPivotServo.getPosition()) <= MotorData.GRIPPER_ERROR;
    }

    @Override
    public void doMovement() {
        if (!initialized) {
            gripperPivotServo.setPosition(targetPosition);
            initialized = true;
        }
    }

    @Override
    public void onStart() {

    }

    @Override
    public void onEnd() {

    }

    @Override
    public Pair<String, String> getStatus() {
        return new Pair<>("Pivot Movement", "Current Pos: " +  gripperPivotServo.getPosition() + "\nTarget Pos: " + this.targetPosition);
    }
}