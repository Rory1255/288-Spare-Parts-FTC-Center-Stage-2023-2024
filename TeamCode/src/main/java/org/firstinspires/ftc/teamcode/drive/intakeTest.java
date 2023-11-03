package org.firstinspires.ftc.teamcode.drive;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.Line;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp(name = "Intake Test", group = "Linear OpMode")
public class intakeTest extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private CRServo topIntakeServo = null;
    private CRServo bottomIntakeServo = null;
    private Servo leftFeedServo = null;
    private Servo rightFeedServo = null;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode(){
        topIntakeServo = hardwareMap.get(CRServo.class, "topIntakeServo");
        bottomIntakeServo = hardwareMap.get(CRServo.class, "bottomIntakeServo");
        leftFeedServo = hardwareMap.get(Servo.class, "leftFeedServo");
        rightFeedServo = hardwareMap.get(Servo.class, "rightFeedServo");

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
             double leftFeedIntake = 0.0;
             double leftFeedStop = 0.5;
             double leftFeedOuttake = 1.0;

             //right feed variables
             double rightFeedIntake = 1.0;
             double rightFeedStop = 0.5;
             double rightFeedOuttake = 0.0;

             //intake control
             if (gamepad2.left_stick_y < 0){
                 topIntakeServo.setPower(topIntake);
                 bottomIntakeServo.setPower(bottomIntake);
             }
             if (gamepad2.left_stick_y == 0){
                 topIntakeServo.setPower(topIntakeStop);
                 bottomIntakeServo.setPower(bottomIntakeStop);
             }
             if (gamepad2.left_stick_y > 0){
                 topIntakeServo.setPower(topOuttake);
                 bottomIntakeServo.setPower(bottomOuttake);
             }

             //feed control
             //feed in
             if (gamepad2.a){
                 leftFeedServo.setPosition(leftFeedIntake);
                 rightFeedServo.setPosition(rightFeedIntake);
             }
             //feed out
             if (gamepad2.x){
                 leftFeedServo.setPosition(leftFeedOuttake);
                 rightFeedServo.setPosition(rightFeedOuttake);
             }
             //feed stop
             if (gamepad2.y) {
                 leftFeedServo.setPosition(leftFeedStop);
                 rightFeedServo.setPosition(rightFeedStop);
             }
        }
    }
}
