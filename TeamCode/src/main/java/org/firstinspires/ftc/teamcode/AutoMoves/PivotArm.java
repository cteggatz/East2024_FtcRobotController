package org.firstinspires.ftc.teamcode.AutoMoves;

import android.util.Pair;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.MotorData;

public class PivotArm extends AutoModeMovements{
    private DcMotor pivotMotor;
    int targetPosition;
    float direction;


    public PivotArm(DcMotor pivotMotor, int targetPosition){
        this.pivotMotor = pivotMotor;
        this.targetPosition = targetPosition;

        //direction = ((pivotMotor.getCurrentPosition() >= targetPosition) ? -1 : 1);
    }



    @Override
    public boolean isDone() {
        //telemetry.addData("isDone", "" + Math.abs(targetPosition - pivotMotor.getCurrentPosition()));
        return Math.abs(targetPosition - pivotMotor.getCurrentPosition()) <= MotorData.PIVOT_ERROR_COUNT;
    }

    @Override
    public void doMovement() {
        direction = ((pivotMotor.getCurrentPosition() >= targetPosition) ? -1 : 1);
        int pivotPosition = pivotMotor.getCurrentPosition();
        double pivotPower = 0;

        pivotPower += direction * MotorData.PIVOT_SPEED_MULT;

        if (MotorData.MAX_COUNT-pivotPosition < MotorData.CUTOFF_COUNT && pivotPower > 0 && MotorData.CUTOFF_PROPORTION > 0) {
            pivotPower *= (MotorData.MAX_COUNT-pivotPosition)/((double)MotorData.CUTOFF_COUNT);
        }

        if (pivotPosition-MotorData.MIN_COUNT < MotorData.CUTOFF_COUNT && pivotPower < 0 && MotorData.CUTOFF_PROPORTION > 0) {
            pivotPower *= (pivotPosition-MotorData.MIN_COUNT)/((double)MotorData.CUTOFF_COUNT);
        }

        pivotMotor.setPower(Range.clip(pivotPower,-1,1) * .5);
        // telemetry.addData("AutoMove", "Moving Pivot");
    }

    @Override
    public void onStart() {

    }

    @Override
    public void onEnd() {
        pivotMotor.setPower(0);
    }

    @Override
    public Pair<String, String> getStatus() {
        return new Pair<>("Pivot Movement", "Current Pos: " +  pivotMotor.getCurrentPosition() + "\nTarget Pos: " + this.targetPosition);
    }

    private int getTargetPosition(){
        return this.targetPosition;
    }
}