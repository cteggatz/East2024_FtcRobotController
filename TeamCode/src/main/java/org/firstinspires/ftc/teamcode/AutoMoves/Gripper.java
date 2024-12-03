package org.firstinspires.ftc.teamcode.AutoMoves;

import android.util.Pair;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MotorData;

public class Gripper extends AutoModeMovements{
    private Servo gripperServo;
    double targetPosition;
    boolean initialized = false;


    public Gripper(Servo gripperServo, double targetPosition){
        this.gripperServo = gripperServo;
        this.targetPosition = targetPosition * .9;
    }

    @Override
    public boolean isDone() {
        //telemetry.addData("isDone", "" + Math.abs(targetPosition - pivotMotor.getCurrentPosition()));
        return Math.abs(targetPosition - gripperServo.getPosition()) <= MotorData.GRIPPER_ERROR;
    }

    @Override
    public void doMovement() {
        if (!initialized) {
            gripperServo.setPosition(targetPosition);
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
        return new Pair<>("Pivot Movement", "Current Pos: " +  gripperServo.getPosition() + "\nTarget Pos: " + this.targetPosition);
    }
}