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
    private DcMotor climbHoist;

    private Servo gripperLeft;
    private Servo gripperRight;

    private int pivotHome = 0;
    private int pivotTarget = -95;
    private int pivotClimbTarget = 100


    private final double maxSpeed = 0.625;
    private final double elevatorPivotUpSpeed = 1;  // Full power to lift
    private final double elevatorPivotDownSpeed = 0.4;  //Because Gravity is helping use less power going down
    private final double elevatorPivotCrawlSpeed = 0.05;  //Slow speed so it doesn't crash
    private final double elevatorPivotClimbSpeed = elevatorPivotUpSpeed;

    private final double gripperSpeed = 0.3;

//    private ElapsedTime runtime = new ElapsedTime();

    public void init(){

        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        rearRight = hardwareMap.get(DcMotor.class, "rearRight");

        elevatorPivot = hardwareMap.get(DcMotor.class, "elevatorPivot");

        climbHoist = hardwareMap.get(DcMotor.class, "climbHoist");

        gripperLeft = hardwareMap.get(Servo.class, "gripperLeft");
        gripperRight = hardwareMap.get(Servo.class, "gripperRight");
        // Maps motors to direction of rotation (Left motors are normally always reversed, may need testing)
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.FORWARD);

        elevatorPivot.setDirection(DcMotor.Direction.FORWARD);

        climbHoist.setDirection(DcMotor.Direction.FORWARD);

        gripperLeft.setDirection(Servo.Direction.REVERSE);
        gripperRight.setDirection(Servo.Direction.FORWARD);

        // When no power is set on a motor, brake.
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elevatorPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        climbHoist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Robot Code Initialized");
    }

    // Function to move pivot arm to desired angle at set speed.
    private void moveToPos(double pow, int pos) {
        elevatorPivot.setPower(pow);
        elevatorPivot.setTargetPosition(pos);
        elevatorPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void moveClimber(double climberpow) {
        climbHoist.setpower(climberpow)
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
        boolean pivotClimb = (gamepad2.x);

        boolean closeGrip = (gamepad2.left_bumper);
        boolean openGrip = (gamepad2.right_bumper);

        boolean spinClimber = (gamepad2.b);

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

        //PIVOT CONTROLS
        // Move up to scoring position
        if(pivotUp ) {
            moveToPos(elevatorPivotUpSpeed, pivotTarget);
            // May need to add a timeout here as it appears to take a while to stop.
        }

        // Move down to intake position
        if(pivotReset ) {
            moveToPos(elevatorPivotDownSpeed,pivotHome + 20);
            moveToPos(elevatorPivotCrawlSpeed,pivotHome);
        }

        // Move up to climbing position
        if (pivotClimb ) {
            moveToPos(elevatorPivotClimbSpeed, pivotClimbTarget);
        }

        // Debugging to tell when the moveToPos function is complete
        if (elevatorPivot.isBusy()) {
            telemetry.addData("Still Moving - Current Pivot Motor Encoder Value ", elevatorPivot.getCurrentPosition());
        }

        if (closeGrip) {
            gripperLeft.setPosition(-0.025);
            gripperRight.setPosition(-0.025);
            telemetry.addData("grip closing", gripperRight.getPosition());
            telemetry.update();
        } else if (openGrip){
            gripperLeft.setPosition(0.25); //test for now lol
            gripperRight.setPosition(0.25);
            telemetry.addData("grip opening", gripperRight.getPosition());
            telemetry.update();
        }

          // Climber Control System
          if (spinClimber.isBusy) {
            moveClimber.climberpow(20);
          } else if (!spinClimber.isBusy) {
            moveClimber.climberpow(0);
          }
    }
}