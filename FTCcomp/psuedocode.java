package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp
public class Mecanum2 extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor arm = null; // the arm motor
    public CRServo intake = null; // the active intake servo
    public DcMotor wrist = null; // the wrist servo
    public Servo claw = null;

    @Override
    public void runOpMode() throws InterruptedException {

        double left;
        double right;
        double forward;
        double rotate;
        double max;

        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        arm = hardwareMap.get(DcMotor.class, "arm"); // the arm motor

        // Set the arm motor to BRAKE behavior and RUN_WITHOUT_ENCODER mode
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /* Define and initialize servos. */
        intake = hardwareMap.get(CRServo.class, "intake");
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(DcMotor.class, "wrist");

        waitForStart();

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1) * 2;
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x + rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // Arm movement controls
            if (gamepad1.dpad_up) {
                // arm.setPower(0.75);  // Move arm up
                // Thread.sleep(50);
                // arm.setPower(0);
                System.out.println("move arm up");     // Stop arm
            }
            else if (gamepad1.dpad_down) {
                // arm.setPower(-0.75); // Move arm down
                // Thread.sleep(50);
                // arm.setPower(0);
                System.out.println("move arm up");     // Stop arm
            }

            // Wrist movement controls
            if (gamepad1.cross) {
                // wrist.setPower(-0.75);  // Move wrist up
                // Thread.sleep(50);
                // wrist.setPower(0);     // Stop wrist
                System.out.println("move wrist up");
            }
            else if (gamepad1.triangle) {
                // wrist.setPower(0.75); // Move wrist down
                // Thread.sleep(50);
                // wrist.setPower(0);     // Stop wrist
                System.out.println("move wrist down");

            }

             double spin = Math.abs(gamepad1.left_trigger-0.5);

            if (gamepad1.left_trigger > 0){
                // intake.setPower(spin);
                // Thread.sleep(200);
                System.out.println("spin counterclockwise");
            }
            else if (gamepad1.right_trigger > 0){
                // intake.setPower(-spin);
                // Thread.sleep(200);
                System.out.println("spin clockwise");
            }
            else if ((gamepad1.right_trigger == 0) && (gamepad1.left_trigger==0)){
                // intake.setPower(0.0);
                System.out.println("stay in place");
            }


        }
    }
}