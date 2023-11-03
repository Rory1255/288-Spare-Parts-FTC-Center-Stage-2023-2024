package org.firstinspires.ftc.teamcode.drive;

import static java.lang.Math.max;
import static java.lang.StrictMath.min;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp(name = "Drive Test", group = "Linear OpMode")
public class driveTest extends LinearOpMode {
    //Variable declarations
    private DcMotor leftFrontDriveMotor = null;
    private DcMotor rightFrontDriveMotor = null;
    private DcMotor leftBackDriveMotor = null;
    private DcMotor rightBackDriveMotor = null;

    private DcMotor elevatorMotor = null;

    final double ELEVATOR_HEIGHT_MAX = 4157;
    private double targetElevatorPosition = 0;
    private final ElapsedTime runtime = new ElapsedTime();
    final double ELEVATOR_HEIGHT_SHORT = 978;
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode(){
        //Hardware Mapping
        leftFrontDriveMotor = hardwareMap.get(DcMotor.class, "leftFrontDriveMotor");
        rightFrontDriveMotor = hardwareMap.get(DcMotor.class, "rightFrontDriveMotor");
        leftBackDriveMotor = hardwareMap.get(DcMotor.class, "leftBackDriveMotor");
        rightBackDriveMotor = hardwareMap.get(DcMotor.class, "rightBackDriveMotor");


        elevatorMotor = hardwareMap.get(DcMotor.class, "elevatorMotor");

        //Set motor direction
        leftFrontDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackDriveMotor.setDirection(DcMotor.Direction.REVERSE);

        elevatorMotor.setDirection(DcMotor.Direction.REVERSE);
        elevatorMotor.setTargetPosition(0);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Set Zero Power Behavior
        leftFrontDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Telemetry(information that gets sent to drive station for quick feedback)
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();
        runtime.reset();


        //runtime until driver hits stop
        while (opModeIsActive()) {

            if (gamepad1.a) {
                targetElevatorPosition = ELEVATOR_HEIGHT_SHORT;
            }

            targetElevatorPosition = max(0.0, targetElevatorPosition); // Cannot go below 0
            targetElevatorPosition = min(targetElevatorPosition, ELEVATOR_HEIGHT_MAX); // Cannot go beyond max height

            elevatorMotor.setTargetPosition((int) targetElevatorPosition);
            elevatorMotor.setPower(1.0);

        }
    }
}
