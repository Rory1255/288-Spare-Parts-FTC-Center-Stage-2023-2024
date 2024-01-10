package org.firstinspires.ftc.teamcode.drive;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@TeleOp(name = "Motor Configuration Wiring Tester", group = "Linear OpMode")
public class MotorConfigurationWiringTester extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotor.class, "frontLeftDriveMotor");
        rightFront = hardwareMap.get(DcMotor.class, "frontRightDriveMotor");
        leftBack = hardwareMap.get(DcMotor.class, "backLeftDriveMotor");
        rightBack = hardwareMap.get(DcMotor.class, "backRightDriveMotor");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {

            if (gamepad1.a){
                leftBack.setPower(1.0);
            }

            if (gamepad1.b){
                rightBack.setPower(1.0);
            }
            if (gamepad1.x){
                leftFront.setPower(1.0);
            }
            if (gamepad1.y){
                rightFront.setPower(1.0);
            }
            if (!gamepad1.a && !gamepad1.b && !gamepad1.x && !gamepad1.y){
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);
            }



        }
    }
}
