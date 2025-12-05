package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto_Blue_Meet2", group = "OpMode")
public class Auto_Blue_Meet2 extends LinearOpMode {

    private DcMotor lf = null, lb = null, rf = null, rb = null;
    private DcMotor leftOuttake = null, rightOuttake = null;
    private Servo left_servo, right_servo;
    private ElapsedTime runtime = new ElapsedTime();

    // --- Constants ---
    static final double COUNTS_PER_MOTOR_REV = 537.7;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 3.75;
    static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() {
        // --- Hardware Mapping ---
        lf = hardwareMap.get(DcMotor.class, "front_left");
        lb = hardwareMap.get(DcMotor.class, "back_left");
        rf = hardwareMap.get(DcMotor.class, "front_right");
        rb = hardwareMap.get(DcMotor.class, "back_right");
        left_servo = hardwareMap.get(Servo.class, "left_servo");
        right_servo = hardwareMap.get(Servo.class, "right_servo");
        leftOuttake = hardwareMap.get(DcMotor.class, "left_shooter");
        rightOuttake = hardwareMap.get(DcMotor.class, "right_shooter");

        // --- Motor Directions ---
        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.FORWARD);

        left_servo.setDirection(Servo.Direction.REVERSE);
        right_servo.setDirection(Servo.Direction.FORWARD);
        leftOuttake.setDirection(DcMotor.Direction.FORWARD);
        rightOuttake.setDirection(DcMotor.Direction.REVERSE);

        // --- Reset Encoders ---
        setDriveMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            telemetry.addData("Path", "Starting auto_red");
            telemetry.update();

            // 1️⃣ Drive backward 24 inches
            encoderDrive(DRIVE_SPEED, 31, 30, false);
            sleep(500);

            // 2️⃣ Run the outtake sequence
            runOuttake(0.73, 1000);
            sleep(500);

            // 2️⃣ Run the outtake sequence
            runOuttake(0.73, 1000);
            sleep(500);

            // 3️⃣ Turn right 45 degrees (adjust as needed for your robot)
            encoderDrive(TURN_SPEED, 12, -12, false);
            sleep(500);

            // 4️⃣ Drive forward 12 inches
            encoderDrive(DRIVE_SPEED, 25, 25, false);

            telemetry.addData("Path", "Complete");
            telemetry.update();
        }
    }

    // --- Outtake Function ---
    public void runOuttake(double power, long durationMs) {
        if (opModeIsActive()) {
            telemetry.addData("Action", "Running Outtake");
            telemetry.update();

            leftOuttake.setPower(power);
            rightOuttake.setPower(power);
            sleep(durationMs);


            // Servo sequence
            //First Shot
            left_servo.setPosition(0.15);
            right_servo.setPosition(0.15);
            sleep(500);
            left_servo.setPosition(0.07);
            right_servo.setPosition(0.07);
            sleep(1800);
            //Second Shot
            left_servo.setPosition(0.15);
            right_servo.setPosition(0.15);
            sleep(500);
            left_servo.setPosition(0.07);
            right_servo.setPosition(0.07);
            sleep(1800);
            //Third Shot
            left_servo.setPosition(0.15);
            right_servo.setPosition(0.15);
            sleep(500);
            left_servo.setPosition(0.01);
            right_servo.setPosition(0.01);



            // Stop the outtake motors
            leftOuttake.setPower(0);
            rightOuttake.setPower(0);
        }
    }

    // --- Encoder Drive Function ---
    public void encoderDrive(double speed, double leftInches, double rightInches, boolean runIntake) {
        if (!opModeIsActive()) return;

        // Fix: invert left side movement because left motors are reversed
        int newLeftTarget = lf.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
        int newRightTarget = rf.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

        lf.setTargetPosition(newLeftTarget);
        lb.setTargetPosition(newLeftTarget);
        rf.setTargetPosition(newRightTarget);
        rb.setTargetPosition(newRightTarget);

        setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        lf.setPower(Math.abs(speed));
        lb.setPower(Math.abs(speed));
        rf.setPower(Math.abs(speed));
        rb.setPower(Math.abs(speed));

        while (opModeIsActive() &&
                (lf.isBusy() || lb.isBusy() || rf.isBusy() || rb.isBusy())) {
            telemetry.addData("Running to", "L:%d R:%d", newLeftTarget, newRightTarget);
            telemetry.addData("Currently at", "LF:%d RF:%d", lf.getCurrentPosition(), rf.getCurrentPosition());
            telemetry.update();
        }

        stopDriveMotors();
        setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // --- Helper Functions ---
    private void stopDriveMotors() {
        lf.setPower(0);
        lb.setPower(0);
        rf.setPower(0);
        rb.setPower(0);
    }

    private void setDriveMotorMode(DcMotor.RunMode mode) {
        lf.setMode(mode);
        lb.setMode(mode);
        rf.setMode(mode);
        rb.setMode(mode);
    }
}
