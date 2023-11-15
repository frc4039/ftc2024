package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="2024 Teleop", group="Iterative OpMode")

public class Teleop24 extends OpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor rearLeft;
    private DcMotor rearRight;

    // Define the Motor Maximum Speed
    private final double maxSpeed = 0.625;
    

    @Override
    public void init(){

        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        rearRight = hardwareMap.get(DcMotor.class, "rearRight");

        // Maps motors to direction of rotation (Left motors are normally always reversed, may need testing)
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.FORWARD);

        // When no power is set on a motor, brake.
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.addData("Status", "Robot Code Initialized");
    }

    @Override
    public void start(){
    }

    @Override
    public void loop(){

        // Position of drive motor encoders
        telemetry.addData("Left Front Motor Encoder:", frontLeft.getCurrentPosition());

        telemetry.addData("Left Back Motor Encoder:", rearLeft.getCurrentPosition());

        telemetry.addData("Right Front Motor Encoder:", frontRight.getCurrentPosition());

        telemetry.addData("Right Back Motor Encoder:", rearRight.getCurrentPosition());

        double drive = (gamepad1.left_stick_y);//inverted???
        double strafe = (-gamepad1.left_stick_x);//inverted???
        double turn = (gamepad1.right_stick_x);

        // This is some really janky math that someone implemented back in 2021, but hey, if it works, ¯\_(ツ)_/¯
        // Square the values and keep the sign to give exponentially more weight to the larger values.
        drive = drive * drive * Math.signum(drive);
        strafe = strafe * strafe * Math.signum(strafe);
        turn = turn * turn * Math.signum(turn);

        // Allows for multiple functions to happen at once (eg. drive & strafe, turn & drive, etc)
        // Combine the weighted values from above as a ratio of maxSpeed
        double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1);
        frontLeft.setPower(maxSpeed*(drive - strafe + turn)/denominator);
        frontRight.setPower(maxSpeed*(drive - strafe - turn)/denominator);
        rearLeft.setPower(maxSpeed*(drive + strafe + turn)/denominator);
        rearRight.setPower(maxSpeed*(drive + strafe - turn)/denominator);
    }
}
