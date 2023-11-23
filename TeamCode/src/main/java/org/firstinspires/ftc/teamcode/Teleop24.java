package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="2024 Teleop", group="Iterative OpMode")

public class Teleop24 extends OpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor rearLeft;
    private DcMotor rearRight;
    private DcMotor elevatorPivot;

    private int pivotTarget = 0;


    // I- don't think this does anything????
    private final double maxSpeed = 0.625;
    private final double elevatorPivotSpeed = 0.2;

    @Override
    public void init(){

        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        rearRight = hardwareMap.get(DcMotor.class, "rearRight");

        elevatorPivot = hardwareMap.get(DcMotor.class, "elevatorPivot");

        // Maps motors to direction of rotation (Left motors are normally always reversed, may need testing)
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.FORWARD);

        elevatorPivot.setDirection(DcMotor.Direction.FORWARD);

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

        double drive = (-gamepad1.left_stick_y);//inverted???
        double strafe = (gamepad1.left_stick_x);//inverted???
        double turn = (gamepad1.right_stick_x);//inverted???

        double pivot = (gamepad2.left_stick_y);
        boolean pivotReset = (gamepad2.a);


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
            elevatorPivot.setTargetPosition(pivotTarget);
            elevatorPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            resetRuntime();
            elevatorPivot.setPower(elevatorPivotSpeed);
        } else {
            elevatorPivot.setPower(elevatorPivotSpeed * pivot);
        }
    }
}