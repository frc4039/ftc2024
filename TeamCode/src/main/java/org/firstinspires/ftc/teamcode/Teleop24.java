package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="2024 Teleop", group="Iterative OpMode")

public class Teleop24 extends OpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor rearLeft;
    private DcMotor rearRight;
    private DcMotor elevatorPivot;

    private Servo gripperLeft;
    private Servo gripperRight;

    private int pivotHome = 0;
    private int pivotTarget = 130;


    // I- don't think this does anything????
    private final double maxSpeed = 0.625;
    private final double elevatorPivotSpeed = 0.2;
    private final double gripperSpeed = 0.3;

    private ElapsedTime runtime = new ElapsedTime();

    public void init(){

        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        rearRight = hardwareMap.get(DcMotor.class, "rearRight");

        elevatorPivot = hardwareMap.get(DcMotor.class, "elevatorPivot");

        gripperLeft = hardwareMap.get(Servo.class, "gripperLeft");
        gripperRight = hardwareMap.get(Servo.class, "gripperRight");
        // Maps motors to direction of rotation (Left motors are normally always reversed, may need testing)
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.FORWARD);

        elevatorPivot.setDirection(DcMotor.Direction.REVERSE);

        gripperLeft.setDirection(Servo.Direction.REVERSE);
        gripperRight.setDirection(Servo.Direction.FORWARD);

        // When no power is set on a motor, brake.
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elevatorPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.addData("Status", "Robot Code Initialized");
    }

    @Override
    public void start(){
        //insert something here.
    }

    @Override
    public void loop(){

        // Position of drive motor encoders
        telemetry.addData("Left Front Motor Encoder:", frontLeft.getCurrentPosition());

        telemetry.addData("Left Back Motor Encoder:", rearLeft.getCurrentPosition());

        telemetry.addData("Right Front Motor Encoder:", frontRight.getCurrentPosition());

        telemetry.addData("Right Back Motor Encoder:", rearRight.getCurrentPosition());
        // Position of elevator arm motor encoder
        telemetry.addData("Elevator Arm Position:", elevatorPivot.getCurrentPosition());

        double drive = (gamepad1.left_stick_y);//inverted???
        double strafe = (-gamepad1.left_stick_x);//inverted???
        double turn = (gamepad1.right_stick_x);//inverted???

        boolean pivotUp = (gamepad2.y);
        boolean pivotReset = (gamepad2.a);

        boolean closeGrip = (gamepad2.left_bumper);
        boolean openGrip = (gamepad2.right_bumper);

        /**
         </>his is some really janky math that someone implemented back in 2021, but hey, if it works, ¯\_(ツ)_/¯
         drive = drive * drive * Math.signum(drive);
         strafe = strafe * strafe * Math.signum(strafe);
         turn = turn * turn * Math.signum(turn);
         **/


        //slightly less janky math that could work

        drive = drive * Math.abs(drive);
        strafe = strafe * Math.abs(strafe);
        turn = turn * Math.abs(turn);


        // Allows for multiple functions to happen at once (eg. drive & strafe, turn & drive, etc)
        double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1);
        frontLeft.setPower(maxSpeed*(drive - strafe + turn)/denominator);
        frontRight.setPower(maxSpeed*(drive - strafe - turn)/denominator);
        rearLeft.setPower(maxSpeed*(drive + strafe + turn)/denominator);
        rearRight.setPower(maxSpeed*(drive + strafe - turn)/denominator);

        if (pivotReset) {
            //idk, get the position of the elevator pivot to move towards 0 somehow??????
            elevatorPivot.setTargetPosition(pivotHome);
            elevatorPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            elevatorPivot.setPower(elevatorPivotSpeed);
            telemetry.addData("Running pivotReset: ", elevatorPivot.getCurrentPosition());
            telemetry.addData("Running to:", pivotHome);
        } else if (pivotUp) {
            elevatorPivot.setTargetPosition(pivotTarget);
            elevatorPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            elevatorPivot.setPower(elevatorPivotSpeed);
            //elevatorPivot.getCurrentPosition() = 130

            //while(elevatorPivot.isBusy()) { //used to have while(opmodeIsActive()) idk if that breaks anything
            telemetry.addData("Running pivotUp: ", elevatorPivot.getCurrentPosition());
            telemetry.addData("running to:", pivotTarget);
            telemetry.update();
            //}
        }

        elevatorPivot.setPower(0);

        if (closeGrip) {
            gripperLeft.setPosition(0);
            gripperRight.setPosition(0);
            telemetry.addData("grip closing", gripperRight.getPosition());
            telemetry.update();
        } else if (openGrip){
            gripperLeft.setPosition(0.25); //test for now lol
            gripperRight.setPosition(0.25);
            telemetry.addData("grip opening", gripperRight.getPosition());
            telemetry.update();
        }

    }
}