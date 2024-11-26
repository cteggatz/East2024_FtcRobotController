package org.firstinspires.ftc.teamcode;

public class MotorData {
    ////////// Movement Constants //////////
    public static final double MIN_DEGREE = -95;
    public static final double MAX_DEGREE = 10;
    public static final double CUTOFF_PROPORTION = 0.05;
    public static final double PIVOT_SPEED_MULT = 0.8;

    public static final int COUNT_PER_REV = 28;
    public static final double GEAR_REDUCTION = 60*125/15;
    public static final double DEGREE_ADJUST = 0.78;

    public static final double COUNT_PER_DEGREE = DEGREE_ADJUST * COUNT_PER_REV * GEAR_REDUCTION / 360;
    public static final double MIN_COUNT = (MIN_DEGREE*COUNT_PER_DEGREE);
    public static final double MAX_COUNT = (MAX_DEGREE*COUNT_PER_DEGREE);
    public static final double CUTOFF_COUNT = CUTOFF_PROPORTION*(MAX_COUNT-MIN_COUNT);


    public static final double DRIVE_MOTOR_SPEED_MIN = 0.1;
    public static final double DRIVE_MOTOR_SPEED_MAX = 1.0;
    public static final int DRIVE_MOTOR_SPEED_LEVELS = 4;
    public static final double BUMPER_DEBOUNCE_TIME  = 10.0; // Minimum number of milliseconds between bumper presses

    // The calculated speed difference between each speed level
    public static final double DRIVE_MOTOR_SPEED_DIFF = DRIVE_MOTOR_SPEED_LEVELS == 1 ?  0 : (DRIVE_MOTOR_SPEED_MAX - DRIVE_MOTOR_SPEED_MIN) / (DRIVE_MOTOR_SPEED_LEVELS - 1);

    ////////// PIVOT VARIABLES //////////
    public static final double PIVOT_ERROR_DEGREE = 3;
    public static final double PIVOT_ERROR_COUNT = PIVOT_ERROR_DEGREE * COUNT_PER_DEGREE;


    ////////// LIFT MOTOR VARIABLES //////////
    public static final float LIFT_MIN_ROTATION = 0;
    public static final float LIFT_MAX_ROTATION = 3000;
    public static final double LIFT_ERROR = 3;

    ////////// GRIPPER //////////
    public static final double GRIPPER_ERROR = 0.05;
}
