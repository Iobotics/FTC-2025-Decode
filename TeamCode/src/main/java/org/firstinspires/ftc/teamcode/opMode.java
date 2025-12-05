package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.util.Range.clip;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Basic: Omni Linear OpMode", group = "Linear OpMode")
public class opmode extends LinearOpMode {

    // Drive motors
    private DcMotor lf, lb, rf, rb;

    // Other motors
    private DcMotor left_shooter, right_shooter;

    // Servos
    Servo left_servo;
    Servo right_servo;

    // Vision
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;

    // Constants for AprilTag alignment
    static final double STRAFE_KP = 0.7;       // left/right strength
    static final double DRIVE_KP = 0.5;        // forward/back strength
    static final double MAX_PWR = 0.35;        // safety limit
    static final double TARGET_DISTANCE = 0.50; // meters

    private boolean isReversed = false;

    @Override
    public void runOpMode() {

        // -------------------------
        // Hardware Init
        // -------------------------
        lf = hardwareMap.get(DcMotor.class, "front_left");
        lb = hardwareMap.get(DcMotor.class, "back_left");
        rf = hardwareMap.get(DcMotor.class, "front_right");
        rb = hardwareMap.get(DcMotor.class, "back_right");

        left_shooter = hardwareMap.get(DcMotor.class, "left_shooter");
        right_shooter = hardwareMap.get(DcMotor.class, "right_shooter");

        left_servo = hardwareMap.get(Servo.class, "left_servo");
        right_servo = hardwareMap.get(Servo.class, "right_servo");

        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.FORWARD);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_shooter.setDirection(DcMotor.Direction.FORWARD);
        right_shooter.setDirection(DcMotor.Direction.REVERSE);

        left_servo.setDirection(Servo.Direction.REVERSE);
        right_servo.setDirection(Servo.Direction.FORWARD);

        // -------------------------
        // AprilTag Vision Init
        // -------------------------
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawCubeProjection(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(CameraName.class, "Webcam 1"))
                .build();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        // -------------------------
        // Main Loop
        // -------------------------
        while (opModeIsActive()) {

            // DRIVER CONTROLS
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double pivot = -gamepad1.right_stick_x;

            double left_shooterp = gamepad2.left_trigger * 0.9;
            double right_shooterp = gamepad2.left_trigger * 0.9;

            // Calc mecanum powers
            double lfp = y + x - pivot;
            double rfp = y - x + pivot;
            double lbp = y - x - pivot;
            double rbp = y + x + pivot;

            double max = Math.max(Math.max(Math.abs(lfp), Math.abs(rfp)),
                    Math.max(Math.abs(lbp), Math.abs(rbp)));

            if (max > 1.0) {
                lfp /= max;
                rfp /= max;
                lbp /= max;
                rbp /= max;
            }

            // Toggle reverse
            if (gamepad1.a) {
                isReversed = !isReversed;
                while (gamepad1.a) {} // wait
            }

            // -------------------------
            // AprilTag Alignment Control
            // -------------------------
            if (gamepad1.left_bumper) {
                aprilTagAlign();
            } else {
                // Normal drive
                if (!isReversed) {
                    lf.setPower(lfp);
                    rf.setPower(rfp);
                    lb.setPower(lbp);
                    rb.setPower(rbp);
                } else {
                    lf.setPower(-lfp);
                    rf.setPower(-rfp);
                    lb.setPower(-lbp);
                    rb.setPower(-rbp);
                }
            }

            // Shooter
            left_shooter.setPower(left_shooterp);
            right_shooter.setPower(right_shooterp);

            telemetry.update();
        }
    }

    // ============================================================
    // APRILTAG ALIGNMENT FUNCTION
    // ============================================================
    public void aprilTagAlign() {

        List<AprilTagDetection> tags = tagProcessor.getDetections();

        if (tags.size() == 0) {
            telemetry.addLine("No Tag Found");
            stopAllDrive();
            return;
        }

        AprilTagDetection tag = tags.get(0);

        double x = tag.ftcPose.x; // left/right (meters)
        double z = tag.ftcPose.z; // forward/back (meters)

        double strafePower = -x * STRAFE_KP;
        double forwardPower = (z - TARGET_DISTANCE) * DRIVE_KP;

        // Safety clamp
        strafePower = clip(strafePower, -MAX_PWR, MAX_PWR);
        forwardPower = clip(forwardPower, -MAX_PWR, MAX_PWR);

        // Deadzones
        if (Math.abs(x) < 0.02) strafePower = 0;
        if (Math.abs(z - TARGET_DISTANCE) < 0.03) forwardPower = 0;

        mecanumDrive(forwardPower, strafePower);

        telemetry.addData("Tag X", x);
        telemetry.addData("Tag Z", z);
        telemetry.addData("Forward Pwr", forwardPower);
        telemetry.addData("Strafe Pwr", strafePower);
    }

    // ============================================================
    // DRIVING HELPERS
    // ============================================================

    public void mecanumDrive(double forward, double strafe) {
        double lfp = forward + strafe;
        double rfp = forward - strafe;
        double lbp = forward - strafe;
        double rbp = forward + strafe;

        lf.setPower(lfp);
        rf.setPower(rfp);
        lb.setPower(lbp);
        rb.setPower(rbp);
    }

    public void stopAllDrive() {
        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
    }
}
