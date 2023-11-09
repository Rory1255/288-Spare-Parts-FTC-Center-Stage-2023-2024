package org.firstinspires.ftc.teamcode.drive;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.Line;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "Intake Test", group = "Linear OpMode")
public class intakeTest extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private CRServo topIntakeServo = null;
    private CRServo bottomIntakeServo = null;
    private CRServo leftFeedServo = null;
    private CRServo rightFeedServo = null;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode(){
        leftFeedServo = hardwareMap.get(CRServo.class, "leftFeedServo");
        rightFeedServo = hardwareMap.get(CRServo.class, "rightFeedServo");

        waitForStart();
        runtime.reset();

        while (opModeIsActive()){
             //Top Intake Servo variables
             double topIntakeStop = 0.0;
             double topIntake = 1.0;
             double topOuttake = -1.0;

             //Bottom Intake Servo variables
             double bottomIntakeStop = 0.0;
             double bottomIntake = 1.0;
             double bottomOuttake = -1.0;

             //left feed variables
             double leftFeedIntake = 1.0;
             double leftFeedStop = 0.0;
             double leftFeedOuttake = -1.0;

             //right feed variables
             double rightFeedIntake = -1.0;
             double rightFeedStop = 0.0;
             double rightFeedOuttake = 1.0;

             //intake control
             /*if (gamepad2.left_stick_y < 0){
                 leftFeedServo.setPower(leftFeedIntake);
                 rightFeedServo.setPower(rightFeedIntake);;
             }
             if (gamepad2.left_stick_y == 0){
                 leftFeedServo.setPower(leftFeedStop);
                 rightFeedServo.setPower(rightFeedStop);
             }
             if (gamepad2.left_stick_y > 0){
                 leftFeedServo.setPower(leftFeedOuttake);
                 rightFeedServo.setPower(rightFeedOuttake);
             }*/


            if (gamepad2.left_trigger == 1.0){
                leftFeedServo.setPower(leftFeedIntake);
                rightFeedServo.setPower(rightFeedIntake);
            }
            if (gamepad2.right_trigger == 1.0){
                leftFeedServo.setPower(leftFeedOuttake);
                rightFeedServo.setPower(rightFeedOuttake);
            }
            if (gamepad2.right_trigger == 0.0 && gamepad2.left_trigger == 0.0){
                leftFeedServo.setPower(leftFeedStop);
                rightFeedServo.setPower(rightFeedStop);
            }

             //feed control
             //feed in
            /* if (gamepad2.a){
                 leftFeedServo.setPower(leftFeedIntake);
                 rightFeedServo.setPower(rightFeedIntake);
             }
             //feed out
             if (gamepad2.x){
                 leftFeedServo.setPower(leftFeedOuttake);
                 rightFeedServo.setPower(rightFeedOuttake);
             }
             //feed stop
             if (gamepad2.y) {
                 leftFeedServo.setPower(leftFeedStop);
                 rightFeedServo.setPower(rightFeedStop);
             }*/
        }
    }
}
