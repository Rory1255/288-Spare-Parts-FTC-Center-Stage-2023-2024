package org.firstinspires.ftc.teamcode.drive;


import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Wheels Go Brrr", group = "Linear OpMode")
public class driveTestForReal extends LinearOpMode {
    //motor variable declarations

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

       //Setting motor direction after empirical testing

        //Telemetry setup to give useful feedback info

        //logic to start program

        //Loop until stop button is pressed
        while (opModeIsActive()){

            //TODO: 1. Make a test to determine proper direction of motors
            //TODO: 2. Make a robot centric drive algorithm
            //TODO: 3. Make a field centric drive algorithm with proper usage of formulas
            //TODO: 4. Make a servo respond to an input
            //TODO: 5. Make a servo respond to an input in a more advanced manner

        }
    }

}

