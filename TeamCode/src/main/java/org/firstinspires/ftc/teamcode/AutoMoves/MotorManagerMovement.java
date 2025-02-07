package org.firstinspires.ftc.teamcode.AutoMoves;

import android.util.Pair;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Managers.MotorManager;

import java.util.Iterator;

public class MotorManagerMovement extends AutoModeMovements implements Iterable<MotorManagerMovement>, Iterator<MotorManagerMovement> {
    MotorManager manager;
    DcMotor motor;
    Telemetry telemetry;
    LinearOpMode opMode;
    double targetRotation;

    /**
     * Creates a new instance of a Motor Manager Movement
     * @param manager the motor manager that will be used in this movement
     * @param motor the motor that the manager will be managing in the movement
     * @param targetPercent the target percentage of the motors rotation range this movement will try to move to
     *
     * @throws IllegalArgumentException when manager is null, motor is null, targetCount is outside bounds of manager
     *
     * @author Christopher Teggatz
     */
    public MotorManagerMovement(MotorManager manager, DcMotor motor, double targetPercent, LinearOpMode opMode){
        if(manager == null)
            throw new IllegalArgumentException("Manager cannot be null");
        if(motor == null)
            throw new IllegalArgumentException("DC Motor cannot be null");
        if(opMode == null)
            throw new IllegalArgumentException("OpMode cannot be null");
        if(targetPercent > 1 || targetPercent < 0)
            throw new IllegalArgumentException("Target Percent must be within [0,1] - provided: " + targetPercent);

        // interpolate the target position depending on the mins and the max of the manager
        this.targetRotation = manager.FromProportionalRotation(targetPercent);

        // set the rest of the variables
        this.manager = manager;
        this.motor = motor;
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
        this.manager.UsingCounts().UpdateRotation(this.motor.getCurrentPosition());
        onStart();
        telemetry.addData("MotorManagerMovement", "Finished Init MotorManagerMovement");
        telemetry.addData("MotorManagerMovement", "TargetPercent: " + targetPercent);
        telemetry.addData("MotorManagerMovement", "TargetRotation: " + targetRotation);
        telemetry.addData("MotorManagerMovement", "CurrentRotation: " + motor.getCurrentPosition());
        telemetry.addData("MotorManagerMovement", "ManagerMax: " + manager.GetMax() + ", ManagerMin: " + manager.GetMin());
        telemetry.update();
    }


    @Override
    public boolean isDone() {
        manager.UpdateRotation(motor.getCurrentPosition());
        return manager.IsNearTargetRotation();
    }

    @Override
    public void doMovement() {
        // updates the managers position and sets the direction of movement
        manager.UpdateRotation(motor.getCurrentPosition());
        manager.SetTargetRotation(targetRotation);
        //manager.SetTargetPower(-1);

        double power = manager.GetFinalPower();
        motor.setPower(power);

        telemetry.addData("MotorManagerMovement", "TargetRotation: " + manager.GetTargetRotation());
        telemetry.addData("MotorManagerMovement", "Power: " + power);
        telemetry.addData("MotorManagerMovement", "Rotation: " + manager.GetRotation());
        telemetry.update();
    }

    @Override
    public void onStart() {

    }

    @Override
    public void onEnd() {
        motor.setPower(0);
        manager.SetTargetPower(0);
        manager.ResetTargetRotation();
    }

    @Override
    public void doLinearMove(){
        onStart();
        while(!isDone() && opMode.opModeIsActive()){
            doMovement();
        }
        onEnd();
        telemetry.addData("MotorManagerMovement", "Finished Movement @ position " + manager.GetRotation());
    }

    @Override
    public Pair<String, String> getStatus() {
        return null;
    }

    /**
     * Returns this instance of the movement so it can be iterated through with an advanced for loop.
     *
     * @return this movement instance
     * @author Christopher Teggatz
     */
    @NonNull
    @Override
    public Iterator<MotorManagerMovement> iterator() {
        onStart();
        return this;
    }

    /**
     *
     *
     * @return true if the movement is done, false if the movement hasn't finished
     */
    @Override
    public boolean hasNext() {
        if(isDone() || !opMode.opModeIsActive()){
            this.onEnd();
            return false;
        } else {
            return true;
        }
    }

    /**
     *
     *
     * @return
     *
     */
    @Override
    public MotorManagerMovement next() {
        doMovement();
        return this;
    }
}
