package org.firstinspires.ftc.teamcode.drive;

import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;
import static java.lang.Math.max;
import static java.lang.Math.min;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@TeleOp(name = "First Comp TeleOp", group = "Linear OpMode")
public class firstCompTeleOp extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDriveMotor = null;
    private DcMotor rightFrontDriveMotor = null;
    private DcMotor leftRearDriveMotor = null;
    private DcMotor rightRearDriveMotor = null;
    private DcMotor armExtensionFront = null;
    private DcMotor armExtensionBack = null;
    private DcMotor armHeightMotor = null;
    private DcMotor airplaneMotor = null;

    //servo
    private CRServo leftBackFeed = null;
    private CRServo rightBackFeed = null;
    private CRServo leftFeedServo = null;
    private CRServo rightFeedServo = null;
    private Servo airplaneServo = null;
    private Servo angleServo = null;

    private ColorSensor frontColor = null;
    private ColorSensor backColor = null;



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


        leftBackFeed = hardwareMap.get(CRServo.class, "backLeftIntakeServo");
        rightBackFeed = hardwareMap.get(CRServo.class, "backRightIntakeServo");
        leftFeedServo = hardwareMap.get(CRServo.class, "frontLeftIntakeServo");
        rightFeedServo = hardwareMap.get(CRServo.class, "frontRightIntakeServo");

        airplaneMotor = hardwareMap.get(DcMotor.class, "airplaneMotor");

        airplaneServo = hardwareMap.get(Servo.class, "airplaneServo");

        frontColor = hardwareMap.get(ColorSensor.class, "frontColor");
        backColor = hardwareMap.get(ColorSensor.class, "backColor");

        angleServo = hardwareMap.get(Servo.class, "angleServo");

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

        armHeightMotor.setDirection(DcMotor.Direction.REVERSE);

        airplaneMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




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


            if (gamepad2.left_stick_y < 0){
                armHeightMotor.setPower(gamepad2.left_stick_y);
            }
            if (gamepad2.left_stick_y == 0){
                armHeightMotor.setPower(0);
            }
            if (gamepad2.left_stick_y > 0){
                armHeightMotor.setPower(gamepad2.left_stick_y);
            }

            if (gamepad2.right_stick_y < 0){
                armExtensionFront.setPower(-gamepad2.right_stick_y);
                armExtensionBack.setPower(-gamepad2.right_stick_y);
            }
            if (gamepad2.right_stick_y == 0){
                armExtensionFront.setPower(0);
                armExtensionBack.setPower(0);
            }
            if (gamepad2.right_stick_y > 0){
                armExtensionFront.setPower(-gamepad2.right_stick_y);
                armExtensionBack.setPower(-gamepad2.right_stick_y);
            }



            //left feed variables
            double leftBackFeedIntake = 0.25;
            double leftBackFeedStop = 0.0;
            double leftBackFeedOuttake = -1.0;

            double leftFeedIntake = 1.0;
            double leftFeedStop = 0.0;
            double leftFeedOuttake = -1.0;

            //right feed variables
            double rightBackFeedIntake = -0.25;
            double rightBackFeedStop = 0.0;
            double rightBackFeedOuttake = 1.0;

            double rightFeedIntake = -1.0;
            double rightFeedStop = 0.0;
            double rightFeedOuttake = 1.0;

            if (backColor.red() > 190 || backColor.blue() > 190 || backColor.green() > 190){
                leftFeedIntake = 0.5;
                rightFeedIntake = -0.6;
                leftBackFeedIntake = 0.0;
                rightBackFeedIntake = 0.0;
                if (frontColor.red() > 190 || frontColor.blue() > 190 || frontColor.green() > 190){
                    leftFeedIntake = 0.0;
                    rightFeedIntake = 0.0;

                }
            }

            if(gamepad2.dpad_down || gamepad1.dpad_down){
                leftFeedServo.setPower(1.0);
                rightFeedServo.setPower(-1.0);
                leftBackFeed.setPower(1.0);
                rightBackFeed.setPower(-1.0);
            }

            //intake control
            if (gamepad2.left_trigger == 1.0){
                leftFeedServo.setPower(leftFeedIntake);
                rightFeedServo.setPower(rightFeedIntake);
                leftBackFeed.setPower(leftBackFeedIntake);
                rightBackFeed.setPower(rightBackFeedIntake);
            }

            if (gamepad2.right_trigger == 1.0){
                leftFeedServo.setPower(leftFeedOuttake);
                rightFeedServo.setPower(rightFeedOuttake);
                leftBackFeed.setPower(leftBackFeedOuttake);
                rightBackFeed.setPower(rightBackFeedOuttake);
            }

            if (gamepad2.right_trigger == 0.0 && gamepad2.left_trigger == 0.0 && !gamepad2.dpad_down && !gamepad1.dpad_down){
                leftFeedServo.setPower(leftFeedStop);
                rightFeedServo.setPower(rightFeedStop);
                leftBackFeed.setPower(leftBackFeedStop);
                rightBackFeed.setPower(rightBackFeedStop);
            }


            double airplaneServoOut = 1.0;
            double airplaneServoStop = 0.5;
            double airplanePower = -1.0;

            if (gamepad2.dpad_up){
                airplaneServo.setPosition(airplaneServoOut);
            }
            if (gamepad2.dpad_up == false){
                airplaneServo.setPosition(airplaneServoStop);
            }

            if (gamepad2.a){
                airplaneMotor.setPower(airplanePower * 0.90);
            }
            if (gamepad2.a == false){
                airplaneMotor.setPower(0.0);
            }

            double intakeAngle = 0.5;
            double outtakeAngle = 0.0;

            if (gamepad2.x){
                angleServo.setPosition(intakeAngle);
            }
            if (gamepad2.b){
                angleServo.setPosition(outtakeAngle);
            }






            //go to target position for height
           /* armHeightMotor.setTargetPosition((int) targetHeight);
            armHeightMotor.setPower(1.0);
            armExtensionFront.setTargetPosition((int) targetLength);
            armExtensionFront.setPower(1.0);
            armExtensionBack.setTargetPosition((int) targetLength);
            armExtensionBack.setPower(1.0);*/


            //Telemetry zone
            telemetry.addData("arm height value", armHeightMotor.getCurrentPosition());
            telemetry.addData("front extension value: ", armExtensionFront.getCurrentPosition());
            telemetry.addData("back extension value: ", armExtensionBack.getCurrentPosition());
            telemetry.addData("left trigger value: ", gamepad2.left_trigger);
            telemetry.addData("right trigger value: ", gamepad2.right_trigger);
            telemetry.addData("airplane Power: ", airplaneMotor.getPower());
            telemetry.addData("Airplane Motor Encoder: ", airplaneMotor.getCurrentPosition());
            telemetry.addData("Front Red: ", frontColor.red());
            telemetry.addData("Front Blue: ", frontColor.blue());
            telemetry.addData("Front Green: ", frontColor.green());
            telemetry.addData("Back Red: ", backColor.red());
            telemetry.addData("Back Blue: ", backColor.blue());
            telemetry.addData("Back Green: ", backColor.green());
            telemetry.addData("Left Odo: ", leftRearDriveMotor.getCurrentPosition());
            telemetry.addData("Middle Odo: ", rightFrontDriveMotor.getCurrentPosition());
            telemetry.addData("Right Odo: ", leftFrontDriveMotor.getCurrentPosition());
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

