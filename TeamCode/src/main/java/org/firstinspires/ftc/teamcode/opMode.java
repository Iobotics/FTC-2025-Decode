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

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lf = null;
    private DcMotor lb = null;
    private DcMotor rf = null;
    private DcMotor rb = null;

    private DcMotor left_shooter = null;
    private DcMotor right_shooter = null;

//    private DcMotor left_intake = null;
//    private DcMotor right_intake = null;

    Servo left_servo;
    Servo right_servo;

    private VisionPortal visionPortal;
    private static AprilTagProcessor tagProcessor;

    static final double STRAFE_KP = 0.01;   // adjust for alignment sensitivity
    static final double DRIVE_KP  = 0.5;    // adjust for distance control
    static final double MAX_PWR   = 0.3;    // safety limit
    static final double TARGET_DISTANCE = 0.5; // meters from tag

    // Tuning




    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        lf = hardwareMap.get(DcMotor.class, "front_left");
        lb = hardwareMap.get(DcMotor.class, "back_left");
        rf = hardwareMap.get(DcMotor.class, "front_right");
        rb = hardwareMap.get(DcMotor.class, "back_right");

        left_shooter = hardwareMap.get(DcMotor.class, "left_shooter");
        right_shooter = hardwareMap.get(DcMotor.class, "right_shooter");

//        left_intake = hardwareMap.get(DcMotor.class, "leftintake");
//        right_intake = hardwareMap.get(DcMotor.class, "rightintake");

        left_servo = hardwareMap.get(Servo.class, "left_servo");
        right_servo = hardwareMap.get(Servo.class, "right_servo");

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
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

//        left_intake.setDirection(DcMotor.Direction.FORWARD);
//        right_intake.setDirection(DcMotor.Direction.REVERSE);

        left_servo.setDirection(Servo.Direction.REVERSE);
        right_servo.setDirection(Servo.Direction.FORWARD);

        double left_intakep;
        double right_intakep;
        boolean isReversed = false;

        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawCubeProjection(true)
                .build();

//        // VisionPortal = manages camera and vision processors
//        visionPortal = new VisionPortal.Builder()
//                .addProcessor(tagProcessor)
//                .setCamera((CameraName) hardwareMap.get("Webcam 1"))
//                .build();

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized :D");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double y = gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double x = -gamepad1.left_stick_x;
            double pivot = -gamepad1.right_stick_x;

            double left_shooterp = gamepad2.left_trigger;
            double right_shooterp = gamepad2.left_trigger;

            left_shooterp *= 0.9;
            right_shooterp *= 0.9;


            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double lfp = y + x - pivot;
            double rfp = y - x + pivot;
            double lbp = y - x - pivot;
            double rbp = y + x + pivot;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            double max = Math.max(Math.max(Math.abs(lfp), Math.abs(rfp)),
                    Math.max(Math.abs(lbp), Math.abs(rbp)));

            if (max > 1.0) {
                lfp /= max;
                rfp /= max;
                lbp /= max;
                rbp /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.


//            frontLeftPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
//            backLeftPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
//            frontRightPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
//            backRightPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad


            // Send calculated power to wheels
            if (gamepad2.y) {
                left_servo.setPosition(0.15);
                right_servo.setPosition(0.15);
                sleep(1000);
            }

            if (gamepad2.right_trigger > 0) {
                left_servo.setPosition(0.01);
                right_servo.setPosition(0.01);
            } else {
                left_servo.setPosition(0.07);
                right_servo.setPosition(0.07);
            }

            if (gamepad2.right_bumper) {
                left_intakep = -0.2;
                right_intakep = -0.2;
            } else if (gamepad2.right_trigger > 0) {
                left_intakep = gamepad2.right_trigger;
                right_intakep = gamepad2.right_trigger;
                left_shooterp = -0.1;
                right_shooterp = -0.1;
            } else {
                left_intakep = 0.0;
                right_intakep = 0.0;
            }


            //boolean lastButtonState = f@TeleOp(name = "Basic: Omni Linear OpMode", group = "Linear OpMode")
            //
            //public class opmode extends LinearOpMode {
            //
            //    // Declare OpMode members for each of the 4 motors.
            //    private ElapsedTime runtime = new ElapsedTime();
            //    private DcMotor lf = null;
            //    private DcMotor lb = null;
            //    private DcMotor rf = null;
            //    private DcMotor rb = null;
            //
            //    private DcMotor left_shooter = null;
            //    private DcMotor right_shooter = null;
            //
            ////    private DcMotor left_intake = null;
            ////    private DcMotor right_intake = null;
            //
            //    Servo left_servo;
            //    Servo right_servo;alse;


            //boolean currentButtonState = gamepad1.a;

            if (gamepad1.a) {
                isReversed = !isReversed;
                while (gamepad1.a) {
                    //wait until released
                }
            }

            // Detect the edge: only toggle when button goes from unpressed to pressed
            //if (currentButtonState && !lastButtonState) {
            //    isReversed = !isReversed;  // Toggle direction
            //}

            // Store current button state for next loop
            //lastButtonState = currentButtonState;

            // Drive logic
            if (isReversed) {
                rb.setPower(-rbp);
                lb.setPower(-lbp);
                rf.setPower(-rfp);
                lf.setPower(-lfp);
            } else {
                lf.setPower(lfp);
                rf.setPower(rfp);
                lb.setPower(lbp);
                rb.setPower(rbp);
            }



            left_shooter.setPower(left_shooterp);
            right_shooter.setPower(right_shooterp);

//            left_intake.setPower(left_intakep/1.5);
//            right_intake.setPower(right_intakep/1.5);


            // Show the elapsed game time and wheel power.

            telemetry.update();


        }



    }
    
}
