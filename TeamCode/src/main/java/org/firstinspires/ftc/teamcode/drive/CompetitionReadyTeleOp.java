package org.firstinspires.ftc.teamcode.drive;

import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;
import static java.lang.Math.max;
import static java.lang.Math.min;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
@Disabled
@TeleOp(name = "Competition Ready TeleOp", group = "Linear OpMode")
public class CompetitionReadyTeleOp extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDriveMotor = null;
    private DcMotor rightFrontDriveMotor = null;
    private DcMotor leftRearDriveMotor = null;
    private DcMotor rightRearDriveMotor = null;
    private DcMotor armExtensionFront = null;
    private DcMotor armExtensionBack = null;
    private DcMotor armHeightMotor = null;

    //servo
    private CRServo topIntakeServo = null;
    private CRServo bottomIntakeServo = null;
    private Servo leftFeedServo = null;
    private Servo rightFeedServo = null;

    private double targetHeight = 0;
    private double maxHeight = 2923;
    private double targetLength = 0;
    private double maxLength = 4200;

    private double scorePresetHeight = 500;
    private double climbPresetHeight = 10;
    final double HEIGHT_ADJUST_RATE = 40.0;
    final double LENGTH_ADJUST_RATE = 55.0;


    //joystick variables
    final double JOYSTICK_DEAD_ZONE = 0.20;
    final double JOYSTICK_MOVEMENT_SENSITIVITY = 0.75;
    final double JOYSTICK_ROTATION_SENSITIVITY = 1.00;

    private BNO055IMU imu;
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        //Hardware mapping
        leftFrontDriveMotor = hardwareMap.get(DcMotor.class, "frontLeftDriveMotor");
        rightFrontDriveMotor = hardwareMap.get(DcMotor.class, "frontRightDriveMotor");
        leftRearDriveMotor = hardwareMap.get(DcMotor.class, "backLeftDriveMotor");
        rightRearDriveMotor = hardwareMap.get(DcMotor.class, "backRightDriveMotor");

        armExtensionFront = hardwareMap.get(DcMotor.class, "frontArmExtensionMotor");
        armExtensionBack = hardwareMap.get(DcMotor.class, "backArmExtensionMotor");
        armHeightMotor = hardwareMap.get(DcMotor.class, "armHeightMotor");

        topIntakeServo = hardwareMap.get(CRServo.class, "topIntakeServo");
        bottomIntakeServo = hardwareMap.get(CRServo.class, "bottomIntakeServo");
        leftFeedServo = hardwareMap.get(Servo.class, "leftFeedServo");
        rightFeedServo = hardwareMap.get(Servo.class, "rightFeedServo");

        //set brake mode
        leftRearDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armHeightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtensionBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtensionFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //motor directions
        leftFrontDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        leftRearDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        rightRearDriveMotor.setDirection(DcMotor.Direction.REVERSE);

        armExtensionFront.setDirection(DcMotor.Direction.FORWARD);
        armExtensionBack.setDirection(DcMotor.Direction.REVERSE);

        armHeightMotor.setDirection(DcMotor.Direction.REVERSE);


        //reset encoders
        armHeightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtensionBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtensionFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set run to position mode
        armHeightMotor.setTargetPosition(0);
        armHeightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armExtensionFront.setTargetPosition(0);
        armExtensionFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armExtensionBack.setTargetPosition(0);
        armExtensionBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        waitForStart();
        runtime.reset();

        //imu parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample OpMode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        //status telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        while (opModeIsActive()) {
            //apply field centric math
            double joystickMovementY = inputScaling(-gamepad1.left_stick_y) * JOYSTICK_MOVEMENT_SENSITIVITY;  // Note: pushing stick forward gives negative value
            double joystickMovementX = inputScaling(gamepad1.left_stick_x) * JOYSTICK_MOVEMENT_SENSITIVITY;
            double yaw = (inputScaling(gamepad1.right_stick_x) * JOYSTICK_ROTATION_SENSITIVITY) * 0.75;

            //get robot orientation from imu
            double robotHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            //input movement values into vector translation in 2d theorem
            double theta = -robotHeading;
            double movementX = joystickMovementX * cos(toRadians(theta)) - joystickMovementY * sin(toRadians(theta));
            double movementY = joystickMovementX * sin(toRadians(theta)) + joystickMovementY * cos(toRadians(theta));

            //logic to reduce speed of robot when left trigger is pressed and return to full speed when released
            if (gamepad1.left_trigger > 0.000) {
                movementX = movementX * 0.45;
                movementY = movementY * 0.45;
                yaw = yaw * 0.45;
            }
            if (gamepad1.left_trigger > 0.000 && gamepad1.left_trigger < 0.001) {
                movementX = movementX / 0.45;
                movementY = movementY / 0.45;
                yaw = yaw / 0.45;
            }

            //set power variables for Mecanum wheels
            double leftFrontPower = (movementY + movementX + yaw);
            double rightFrontPower = (movementY - movementX - yaw);
            double leftBackPower = (movementY - movementX + yaw);
            double rightBackPower = (movementY + movementX - yaw);

            //normalize power variables to prevent motor power from exceeding 1.0
            double maxPower = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            maxPower = Math.max(maxPower, Math.abs(leftBackPower));
            maxPower = Math.max(maxPower, Math.abs(rightBackPower));
            if (maxPower > 1.0) {
                leftFrontPower /= maxPower;
                rightFrontPower /= maxPower;
                leftBackPower /= maxPower;
                rightBackPower /= maxPower;

            }

            //apply calculated motor powers
            leftFrontDriveMotor.setPower(leftFrontPower);
            rightFrontDriveMotor.setPower(rightFrontPower);
            leftRearDriveMotor.setPower(leftBackPower);
            rightRearDriveMotor.setPower(rightBackPower);

            //reset field centric button
            if (gamepad1.y) {
                imu.initialize(parameters);
            }

            //joystick height control
            double heightDelta = (gamepad2.left_stick_y * HEIGHT_ADJUST_RATE);
            targetHeight += heightDelta;

            double lengthDelta = (-gamepad2.right_stick_y * LENGTH_ADJUST_RATE);
            targetLength += lengthDelta;

            //min and max of height
            targetHeight = max(-1130.0, targetHeight);
            targetHeight = min(targetHeight, maxHeight);
            //min and max of length
            targetLength = max(0.0, targetLength);
            targetLength = min(targetLength, maxLength);

            //height preset buttons
            if (gamepad2.b){
                targetHeight = scorePresetHeight;
            }
            if (gamepad2.y){
                targetHeight = climbPresetHeight;
            }




            //intake control zone
            //Top Intake Servo variables
            double topIntakeStop = 0.0;
            double topIntake = 1.0;
            double topOuttake = -1.0;

            //Bottom Intake Servo variables
            double bottomIntakeStop = 0.0;
            double bottomIntake = 1.0;
            double bottomOuttake = -1.0;

            //left feed variables
            double leftFeedIntake = 0.0;
            double leftFeedStop = 0.5;
            double leftFeedOuttake = 1.0;

            //right feed variables
            double rightFeedIntake = 1.0;
            double rightFeedStop = 0.5;
            double rightFeedOuttake = 0.0;

            //intake control
            if (gamepad2.left_trigger > 0.000){
                topIntakeServo.setPower(topIntake);
                bottomIntakeServo.setPower(bottomIntake);
                leftFeedServo.setPosition(leftFeedIntake);
                rightFeedServo.setPosition(rightFeedIntake);
            }
            if (gamepad2.left_trigger == 0.000){
                topIntakeServo.setPower(topIntakeStop);
                bottomIntakeServo.setPower(bottomIntakeStop);
                leftFeedServo.setPosition(leftFeedStop);
                rightFeedServo.setPosition(rightFeedStop);
            }

            if (gamepad2.right_trigger > 0.000){
                topIntakeServo.setPower(topOuttake);
                bottomIntakeServo.setPower(bottomOuttake);
                leftFeedServo.setPosition(leftFeedOuttake);
                rightFeedServo.setPosition(rightFeedOuttake);
            }
            if (gamepad2.right_trigger > 0.000 && gamepad2.right_trigger < 0.001){
                topIntakeServo.setPower(topIntakeStop);
                bottomIntakeServo.setPower(bottomIntakeStop);
                leftFeedServo.setPosition(leftFeedStop);
                rightFeedServo.setPosition(rightFeedStop);
            }


            //go to target position for height
            armHeightMotor.setTargetPosition((int) targetHeight);
            armHeightMotor.setPower(1.0);
            armExtensionFront.setTargetPosition((int) targetLength);
            armExtensionFront.setPower(1.0);
            armExtensionBack.setTargetPosition((int) targetLength);
            armExtensionBack.setPower(1.0);


            //Telemetry zone
            telemetry.addData("arm height value", armHeightMotor.getCurrentPosition());
            telemetry.addData("front extension value: ", armExtensionFront.getCurrentPosition());
            telemetry.addData("back extension value: ", armExtensionBack.getCurrentPosition());
            telemetry.addData("left trigger value: ", gamepad2.left_trigger);
            telemetry.addData("right trigger value: ", gamepad2.right_trigger);
            telemetry.update();
        }
    }
    double inputScaling(double x) {
        double sign = Math.signum(x);
        double magnitude = Math.abs(x);
        if (magnitude < JOYSTICK_DEAD_ZONE) {
            magnitude = 0.0;
        } else {
            magnitude = (magnitude - JOYSTICK_DEAD_ZONE) / (1.0 - JOYSTICK_DEAD_ZONE);
        }
        magnitude = Math.pow(magnitude, 2.0);
        return sign * magnitude;
    }
}

