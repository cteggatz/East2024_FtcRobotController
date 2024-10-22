package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="Auto: Basic OpMode", group="Iterative OpMode")
public class AutoOpMode extends OpMode {

    private class EncoderRequest {
        public double leftInches;
        public double rightInches;
        public double speed;
        public double timeout;

        public EncoderRequest(double leftInches, double rightInches, double speed, double timeout) {
            this.leftInches = leftInches;
            this.rightInches = rightInches;
            this.speed = speed;
            this.timeout = timeout;
        }
    }

    /* Declare OpMode members. */
    private DcMotor         leftDrive   = null;
    private DcMotor         rightDrive  = null;

    private ElapsedTime     runtime = new ElapsedTime();
    private List<EncoderRequest> encoderRequests = new ArrayList<>();
    private double currentRequestTimeout = -1; // In milliseconds

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 28 ;    // REV-41-1291: HD Hex Motor No Gearbox
    static final double     DRIVE_GEAR_REDUCTION    = 20.0 ;     // REV-41-1602: 4:1 and REV-41-1603 5:1
    static final double     WHEEL_DIAMETER_INCHES   = 91.55/25.4 ;     // 90mm diameter wheels are converted to inches for figuring circumference.
    static final double     COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    // DRIVE and TURN proportions reflect how many inches the wheels need to rotate in order to move/turn the robot. Currently determined through trial and error.
    static final double     DRIVE_PROPORTION = 0.935;
    static final double     TURN_PROPORTION       = 0.1185;

    static final double     DRIVE_SPEED             = 1.0;
    static final double     TURN_SPEED              = 1.0;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        // Initialize the drive system variables.
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Move in a square
        for (int i = 0; i < 4; i++) {
            addMove(20);
            addTurn(90);
        }

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d",
                leftDrive.getCurrentPosition(),
                rightDrive.getCurrentPosition());
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        double deltaTime = runtime.milliseconds();
        runtime.reset();

        updateEncoderRequests(deltaTime);

        telemetry.update();
    }

    @Override
    public void stop() {

    }

    /**
     * Updates the movement queue.
     * @param deltaTime How much time has passed since this function was last called, in milliseconds.
     */
    private void updateEncoderRequests(double deltaTime) {
        // If we have some time left, decrease it by the time passed.
        if (currentRequestTimeout > 0) {
            currentRequestTimeout -= deltaTime;
            if (currentRequestTimeout < 0) currentRequestTimeout = 0;
        }

        boolean timedOut = currentRequestTimeout == 0;
        boolean busy = leftDrive.isBusy() || rightDrive.isBusy();

        // Try to handle the next request.
        if (timedOut || !busy) {
            if (!encoderRequests.isEmpty()) {
                // Pop the next request off the list.
                EncoderRequest encoderRequest = encoderRequests.get(0);
                encoderRequests.remove(0);

                // Set target motor position to current plus the desired change.
                int leftTarget = leftDrive.getCurrentPosition() + (int)(encoderRequest.leftInches * COUNTS_PER_INCH);
                int rightTarget = rightDrive.getCurrentPosition() + (int)(encoderRequest.rightInches * COUNTS_PER_INCH);
                leftDrive.setTargetPosition(leftTarget);
                rightDrive.setTargetPosition(rightTarget);

                // Turn on RUN_TO_POSITION
                leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // Start motion.
                leftDrive.setPower(Math.abs(encoderRequest.speed));
                rightDrive.setPower(Math.abs(encoderRequest.speed));

                currentRequestTimeout = encoderRequest.timeout;
            } else {
                // Stop motion;
                leftDrive.setPower(0);
                rightDrive.setPower(0);

                // Turn off RUN_TO_POSITION
                leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        } else {
            // Display it for the driver.
            telemetry.addData("Running to",  " %7d :%7d",
                    leftDrive.getTargetPosition(), rightDrive.getTargetPosition());
            telemetry.addData("Currently at",  " at %7d :%7d",
                    leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
        }
    }

    /**
     * Adds a linear drive request to the movement queue.
     * @param distance How far the robot will move forward, in inches. Negative values move backward.
     * @param speed How fast the robot will move.
     * @param timeout The maximum amount of time the robot will try to move for. -1 is unlimited.
     */
    private void addMove(double distance, double speed, double timeout) {
        encoderRequests.add(new EncoderRequest(
                distance*DRIVE_PROPORTION,
                distance*DRIVE_PROPORTION,
                speed,
                timeout
        ));
    }

    private void addMove(double distance) {
        addMove(distance, DRIVE_SPEED, -1);
    }

    /**
     * Adds a angular drive request to the movement queue.
     * @param angle How far the robot will turn right, in degrees. Negative values turn left.
     * @param speed How fast the robot will turn.
     * @param timeout The maximum amount of time the robot will try to turn for. -1 is unlimited.
     */

    private void addTurn(double angle, double speed, double timeout) {
        encoderRequests.add(new EncoderRequest(
                angle*TURN_PROPORTION,
                -angle*TURN_PROPORTION,
                speed,
                timeout
        ));
    }

    private void addTurn(double angle) {
        addTurn(angle, TURN_SPEED, -1);
    }
}
