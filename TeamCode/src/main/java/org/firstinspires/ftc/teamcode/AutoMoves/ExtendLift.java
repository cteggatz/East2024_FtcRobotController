package org.firstinspires.ftc.teamcode.AutoMoves;

import android.util.Pair;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MotorData;

public class ExtendLift extends  AutoModeMovements{
    DcMotor liftMotor;
    int targetPosition;
    float direction;

    public ExtendLift(DcMotor liftMotor, int targetPosition){
        this.liftMotor = liftMotor;
        this.targetPosition = targetPosition;
    }


    @Override
    public boolean isDone() {
        return Math.abs(targetPosition - liftMotor.getCurrentPosition()) <= MotorData.LIFT_ERROR;
    }

    @Override
    public void doMovement() {
        direction = ((liftMotor.getCurrentPosition() >= targetPosition) ? -1 : 1);

        float currentPosition = liftMotor.getCurrentPosition();
        if((currentPosition <= MotorData.LIFT_MAX_ROTATION && direction > 0) || (currentPosition >= MotorData.LIFT_MIN_ROTATION && direction < 0)){
            liftMotor.setPower(direction);
        } else {
            liftMotor.setPower(0);
        }

    }

    @Override
    public void onStart() {

    }

    @Override
    public void onEnd() {
        liftMotor.setPower(0);
    }

    @Override
    public Pair<String, String> getStatus() {
        return new Pair<>("Extend Lift Movement", "Current Pos: " + liftMotor.getCurrentPosition() + "\nTargetPos: " + this.targetPosition);
    }
}