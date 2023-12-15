package org.firstinspires.ftc.teamcode.drive;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name= "Close Blue", group= "Linear OpMode")
public class CloseBlueJankDriveByTime extends LinearOpMode {

    /* Declare OpMode members. */
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDriveMotor = null;
    private DcMotor rightFrontDriveMotor = null;
    private DcMotor leftRearDriveMotor = null;
    private DcMotor rightRearDriveMotor = null;

    private DcMotor armExtensionFront = null;
    private DcMotor armExtensionBack = null;
    private DcMotor armHeightMotor = null;
    private CRServo leftBackFeed = null;
    private CRServo rightBackFeed = null;
    private CRServo leftFeedServo = null;
    private CRServo rightFeedServo = null;
    private Servo angleServo = null;

    double forwardSpeed = 0.5;
    double backwardSpeed = -0.5;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        leftFrontDriveMotor = hardwareMap.get(DcMotor.class, "frontLeftDriveMotor");
        rightFrontDriveMotor = hardwareMap.get(DcMotor.class, "frontRightDriveMotor");
        leftRearDriveMotor = hardwareMap.get(DcMotor.class, "backLeftDriveMotor");
        rightRearDriveMotor = hardwareMap.get(DcMotor.class, "backRightDriveMotor");

        armExtensionFront = hardwareMap.get(DcMotor.class, "frontArmExtensionMotor");
        armExtensionBack = hardwareMap.get(DcMotor.class, "backArmExtensionMotor");
        armHeightMotor = hardwareMap.get(DcMotor.class, "armHeightMotor");


        leftBackFeed = hardwareMap.get(CRServo.class, "backLeftIntakeServo");
        rightBackFeed = hardwareMap.get(CRServo.class, "backRightIntakeServo");
        leftFeedServo = hardwareMap.get(CRServo.class, "frontLeftIntakeServo");
        rightFeedServo = hardwareMap.get(CRServo.class, "frontRightIntakeServo");

        angleServo = hardwareMap.get(Servo.class, "angleServo");
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        //set brake mode
        leftRearDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        //motor directions
        leftFrontDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        leftRearDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        rightRearDriveMotor.setDirection(DcMotor.Direction.REVERSE);

        armExtensionFront.setDirection(DcMotor.Direction.FORWARD);
        armExtensionBack.setDirection(DcMotor.Direction.REVERSE);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Strafe left for however long until robot is aligned with backdrop
        leftFrontDriveMotor.setPower(forwardSpeed);
        leftRearDriveMotor.setPower(backwardSpeed);
        rightFrontDriveMotor.setPower(backwardSpeed);
        rightRearDriveMotor.setPower(forwardSpeed);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.5)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        leftFrontDriveMotor.setPower(0);
        leftRearDriveMotor.setPower(0);
        rightFrontDriveMotor.setPower(0);
        rightRearDriveMotor.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Pause", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 2:  Drive Forward for a brief moment to reduce distance arm must travel
        leftFrontDriveMotor.setPower(forwardSpeed);
        leftRearDriveMotor.setPower(forwardSpeed);
        rightFrontDriveMotor.setPower(forwardSpeed);
        rightRearDriveMotor.setPower(forwardSpeed);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.7)) {
            telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        leftFrontDriveMotor.setPower(0);
        leftRearDriveMotor.setPower(0);
        rightFrontDriveMotor.setPower(0);
        rightRearDriveMotor.setPower(0);
        armHeightMotor.setPower(0.8);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.1)) {
            telemetry.addData("Pause", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 3:  Set intake servo angle to score on board and extend arm a small distance
        armHeightMotor.setPower(0);
        angleServo.setPosition(0.0);
        armExtensionFront.setPower(1.0);
        armExtensionBack.setPower(1.0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        double leftFeedOuttake = 0.8;
        double rightFeedOuttake = -0.8;
        armExtensionFront.setPower(0);
        armExtensionBack.setPower(0);
        leftFeedServo.setPower(leftFeedOuttake);
        leftBackFeed.setPower(leftFeedOuttake);
        rightFeedServo.setPower(rightFeedOuttake);
        rightBackFeed.setPower(rightFeedOuttake);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.3)) {
            telemetry.addData("Action", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        leftFeedServo.setPower(0);
        leftBackFeed.setPower(0);
        rightFeedServo.setPower(0);
        rightBackFeed.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.8)) {
            telemetry.addData("Action", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        leftFeedServo.setPower(leftFeedOuttake);
        leftBackFeed.setPower(leftFeedOuttake);
        rightFeedServo.setPower(rightFeedOuttake);
        rightBackFeed.setPower(rightFeedOuttake);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Action", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //Step 4:  Retract arm
        armExtensionFront.setPower(-1.0);
        armExtensionBack.setPower(-1.0);
        leftFeedServo.setPower(0);
        leftBackFeed.setPower(0);
        rightFeedServo.setPower(0);
        rightBackFeed.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }



        //Step 5: stop arm retraction and strafe right a small amount
        armExtensionFront.setPower(0);
        armExtensionBack.setPower(0);
        leftFrontDriveMotor.setPower(forwardSpeed);
        leftRearDriveMotor.setPower(backwardSpeed);
        rightFrontDriveMotor.setPower(backwardSpeed);
        rightRearDriveMotor.setPower(forwardSpeed);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.8)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        // Step 6:  Stop
        leftFrontDriveMotor.setPower(0);
        leftRearDriveMotor.setPower(0);
        rightFrontDriveMotor.setPower(0);
        rightRearDriveMotor.setPower(0);


        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}