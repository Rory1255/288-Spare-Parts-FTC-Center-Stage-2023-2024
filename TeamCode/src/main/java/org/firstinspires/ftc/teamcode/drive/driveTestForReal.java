package org.firstinspires.ftc.teamcode.drive;


import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Wheels Go Brrr", group = "Linear OpMode")
public class driveTestForReal extends LinearOpMode {
    //motor variable declarations
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDriveMotor = null;
    private DcMotor rightFrontDriveMotor = null;
    private DcMotor leftRearDriveMotor = null;
    private DcMotor rightRearDriveMotor = null;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode(){

        //Hardware mapping
       leftFrontDriveMotor = hardwareMap.get(DcMotor.class, "leftFrontDriveMotor");
       rightFrontDriveMotor = hardwareMap.get(DcMotor.class, "rightFrontDriveMotor");
       leftRearDriveMotor = hardwareMap.get(DcMotor.class, "Leftreardrivemotor");
       rightRearDriveMotor = hardwareMap.get(DcMotor.class, "Rightreardrivemotor");

       //Setting zero power behavior
       leftRearDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       rightRearDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       leftFrontDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       rightFrontDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

       //Setting motor direction after empirical testing
        leftFrontDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        leftRearDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        rightRearDriveMotor.setDirection(DcMotor.Direction.REVERSE);

        //Telemetry setup to give useful feedback info

        //logic to start program

        waitForStart();
        runtime.reset();
        //Loop until stop button is pressed
        while (opModeIsActive()){


            //TODO: 2. Make a robot centric drive algorithm
            //TODO: 3. Make a field centric drive algorithm with proper usage of formulas
            //TODO: 4. Make a servo respond to an input
            //TODO: 5. Make a servo respond to an input in a more advanced manner
            double yAxis = -gamepad1.left_stick_y;
            double xAxis = gamepad1.left_stick_x;
            double rotX = gamepad1.right_stick_x;

            double leftFrontPower = yAxis + xAxis + rotX;
            double rightFrontPower = yAxis - xAxis + rotX;
            double leftBackPower =  yAxis - xAxis + rotX;
            double rightBackPower = yAxis + xAxis - rotX;

            double max;
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            leftFrontDriveMotor.setPower(leftFrontPower);
            rightFrontDriveMotor.setPower(rightFrontPower);
            leftRearDriveMotor.setPower(leftBackPower);
            rightRearDriveMotor.setPower(rightBackPower);




        }
    }

}

