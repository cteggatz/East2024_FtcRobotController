package org.firstinspires.ftc.teamcode.Macros;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.AutoMoves.AutoModeMovements;
import org.firstinspires.ftc.teamcode.AutoMoves.ExtendLift;
import org.firstinspires.ftc.teamcode.AutoMoves.MultiMove;
import org.firstinspires.ftc.teamcode.AutoMoves.PivotArm;
import org.firstinspires.ftc.teamcode.AutoMoves.QueueMove;
import org.firstinspires.ftc.teamcode.MotorData;

public class PutInBasket extends MultiMove {
    public PutInBasket(DcMotor pivotMotor, DcMotor liftMotor) {
        super(new AutoModeMovements[]{
                new PivotArm(pivotMotor, (int)MotorData.MIN_COUNT),
                new ExtendLift(liftMotor, 2800)

        });
    }
}
