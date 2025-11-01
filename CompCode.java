package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * Copyright (c) 2025 Base 10 Assets, LLC
 * [License header same as StarterBotAuto.java]
 */






@Autonomous(name="StarterBotAutoMecanums", group="StarterBot")
public class StarterBotAutoMecanums extends OpMode {

    // Constants same as StarterBotAuto.java
    final double FEED_TIME = 0.20;
    final double LAUNCHER_TARGET_VELOCITY = 1125;
    final double LAUNCHER_MIN_VELOCITY = 1075;
    final double TIME_BETWEEN_SHOTS = 2;
    final double DRIVE_SPEED = 0.75;
    final double ROTATE_SPEED = 0.25;
    final double WHEEL_DIAMETER_MM = 100;
    final double ENCODER_TICKS_PER_REV = 537.7;
    final double TICKS_PER_MM = (ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI));
    final double TRACK_WIDTH_MM = 404;

    int shotsToFire = 3;
    double robotRotationAngle = 45;

    private ElapsedTime shotTimer = new ElapsedTime();
    private ElapsedTime feederTimer = new ElapsedTime();
    private ElapsedTime driveTimer = new ElapsedTime();

    // Updated to use 4 drive motors instead of 2
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null; 
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    // Same enum definitions as StarterBotAuto.java
    private enum LaunchState {
        IDLE,
        PREPARE,
        LAUNCH,
    }

    private enum AutonomousState {
        LAUNCH,
        WAIT_FOR_LAUNCH,
        DRIVING_AWAY_FROM_GOAL,
        ROTATING,
        DRIVING_OFF_LINE,
        COMPLETE;
    }

    private enum Alliance {
        RED,
        BLUE;
    }

    private LaunchState launchState;
    private AutonomousState autonomousState;
    private Alliance alliance = Alliance.RED;

    @Override
    public void init() {
        autonomousState = AutonomousState.WAIT_FOR_LAUNCH;
        launchState = LaunchState.LAUNCH;

        // Initialize hardware with 4 drive motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        // Set motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Reset encoders for all 4 drive motors
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set zero power behavior for all motors
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);

        // Launcher setup same as original
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, 
            new PIDFCoefficients(300,0,0,10));

        leftFeeder.setDirection(CrServo.Direction.REVERSE);
        
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
        rightFeeder.setPower(0);
        leftFeeder.setPower(0);

        if (gamepad1.x) {
            alliance = Alliance.BLUE;
        } else if (gamepad1.b) {
            alliance = Alliance.RED;
        }

        telemetry.addData("Press X", "for BLUE");
        telemetry.addData("Press B", "for RED");
        telemetry.addData("Selected Alliance", alliance);
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        switch (autonomousState) {
            case LAUNCH:
                if(launch(true)) {
                    shotTimer.reset();
                    autonomousState = AutonomousState.WAIT_FOR_LAUNCH;
                }
                break;

            case WAIT_FOR_LAUNCH:
                if(shotTimer.seconds() > TIME_BETWEEN_SHOTS) {
                    shotsToFire-= 1;
                    if(shotsToFire > 0) {
                        autonomousState = AutonomousState.LAUNCH;
                    } else {
                        autonomousState = AutonomousState.WAIT_FOR_LAUNCH;
                    }
                }
                break;

            case DRIVING_AWAY_FROM_GOAL:
                if(drive(DRIVE_SPEED, -500, DistanceUnit.MM, 0.5)) {
                    autonomousState = AutonomousState.ROTATING;
                }
                break;

            case ROTATING:
                if(rotate(ROTATE_SPEED, robotRotationAngle, AngleUnit.DEGREES, 0.5)) {
                    autonomousState = AutonomousState.DRIVING_OFF_LINE;
                }
                break;

            case DRIVING_OFF_LINE:
                if(drive(DRIVE_SPEED, -500, DistanceUnit.MM, 0.5)) {
                    autonomousState = AutonomousState.COMPLETE;
                }
                break;
        }

        telemetry.addData("AutoState", autonomousState);
        telemetry.addData("LauncherState", launchState);
        telemetry.addData("Motor Positions", "LF(%d), RF(%d), LB(%d), RB(%d)",
                leftFrontDrive.getCurrentPosition(), 
                rightFrontDrive.getCurrentPosition(),
                leftBackDrive.getCurrentPosition(),
                rightBackDrive.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
    }

    boolean launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                    launchState = LaunchState.IDLE;
                }
                break;

            case PREPARE:
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    leftFeeder.setPower(1);
                    rightFeeder.setPower(1);
                    feederTimer.reset();
                    launchState = LaunchState.PREPARE;
                }
                break;

            case LAUNCH:
                if (feederTimer.seconds() > FEED_TIME) {
                    leftFeeder.setPower(0);
                    rightFeeder.setPower(0);
                    launchState = LaunchState.LAUNCH;
                    return true;
                }
                break;
        }
        return false;
    }

    boolean drive(double speed, double distance, DistanceUnit distanceUnit, double holdSeconds) {
        final double TOLERANCE_MM = 10;
        
        double targetPosition = (distanceUnit.toMm(distance) * TICKS_PER_MM);

        // Set target position for all 4 motors
        leftFrontDrive.setTargetPosition((int) targetPosition);
        rightFrontDrive.setTargetPosition((int) targetPosition);
        leftBackDrive.setTargetPosition((int) targetPosition);
        rightBackDrive.setTargetPosition((int) targetPosition);

        // Set run mode for all 4 motors
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set power for all 4 motors
        leftFrontDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightBackDrive.setPower(speed);

        if(Math.abs(targetPosition - leftFrontDrive.getCurrentPosition()) > (TOLERANCE_MM * TICKS_PER_MM)) {
            driveTimer.reset();
            autonomousState = AutonomousState.WAIT_FOR_LAUNCH;
            launchState = LaunchState.LAUNCH;
            return false;
        }

        return (driveTimer.seconds() > holdSeconds);
    }

    boolean rotate(double speed, double angle, AngleUnit angleUnit, double holdSeconds) {
        final double TOLERANCE_MM = 10;
        
        if(alliance == Alliance.RED){
            angle -= 90;
        }
        // Convert angle to distance each wheel needs to travel
        double arcLength = (angleUnit.toRadians(angle) * TRACK_WIDTH_MM / 2.0);
        double targetPosition = arcLength * TICKS_PER_MM;

        // Set opposite directions for rotation
        leftFrontDrive.setTargetPosition((int) targetPosition);
        leftBackDrive.setTargetPosition((int) targetPosition);
        rightFrontDrive.setTargetPosition((int) -targetPosition);
        rightBackDrive.setTargetPosition((int) -targetPosition);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightBackDrive.setPower(speed);

        if(Math.abs(targetPosition - leftFrontDrive.getCurrentPosition()) > (TOLERANCE_MM * TICKS_PER_MM)) {
            driveTimer.reset();
            autonomousState = AutonomousState.WAIT_FOR_LAUNCH;
            launchState = LaunchState.LAUNCH;
            return false;
        }

        return (driveTimer.seconds() > holdSeconds);
    }
}
