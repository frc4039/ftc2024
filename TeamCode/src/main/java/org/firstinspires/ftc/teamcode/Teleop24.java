package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="2024 Teleop", group="Iterative OpMode")

public class Teleop24 extends OpMode {

    private boolean UpPos; //true is up, false is down

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor rearLeft;
    private DcMotor rearRight;
    
    private DcMotor elevatorPivot;
    private DcMotor climbHoist;

    private Servo gripperLeft;
    private Servo gripperRight;

    private Servo purplePixelGripper;
    private Servo droneLauncher;

    private int pivotHome = 0;
    private int pivotTarget = -95;
    private int pivotClimbTarget = -190;


    private final double maxSpeed = 0.625;
    private final double elevatorPivotUpSpeed = 1;  // Full power to lift
    private final double elevatorPivotDownSpeed = 0.4;  //Because Gravity is helping use less power going down
    private final double elevatorPivotCrawlSpeed = 0.05;  //Slow speed so it doesn't crash
    private final double elevatorPivotClimbSpeed = 0.6;

    private final double gripperSpeed = 0.3;

//    private ElapsedTime runtime = new ElapsedTime();

    public void init(){
        UpPos = false;

        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        rearRight = hardwareMap.get(DcMotor.class, "rearRight");

        elevatorPivot = hardwareMap.get(DcMotor.class, "elevatorPivot");

        climbHoist = hardwareMap.get(DcMotor.class, "climbHoist");

        gripperLeft = hardwareMap.get(Servo.class, "gripperLeft");
        gripperRight = hardwareMap.get(Servo.class, "gripperRight");

        purplePixelGripper = hardwareMap.get(Servo.class, "purplePixelGripper");
        droneLauncher = hardwareMap.get(Servo.class, "droneLauncher");
        // Maps motors to direction of rotation (Left motors are normally always reversed, may need testing)
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.FORWARD);

        elevatorPivot.setDirection(DcMotor.Direction.FORWARD);
        elevatorPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatorPivot.setPower(0.0);
//        elevatorPivot.setTargetPosition(0);
//        elevatorPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        climbHoist.setDirection(DcMotor.Direction.FORWARD);

        gripperLeft.setDirection(Servo.Direction.REVERSE);
        gripperRight.setDirection(Servo.Direction.FORWARD);

        purplePixelGripper.setDirection(Servo.Direction.REVERSE);
        droneLauncher.setDirection(Servo.Direction.REVERSE);

        purplePixelGripper.setPosition(0.0);
        droneLauncher.setPosition(0.0);

        // When no power is set on a motor, brake.
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
  //      moveToPos(elevatorPivotCrawlSpeed, pivotHome);

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
        climbHoist.setPower(climberpow);
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

        telemetry.addData("Purple pixel servo position:", purplePixelGripper.getPosition());
        telemetry.addData("Drone launcher servo position:", droneLauncher.getPosition());

        double drive = (-gamepad1.left_stick_y);//inverted???
        double strafe = (gamepad1.left_stick_x);//inverted???
        double turn = (gamepad1.right_stick_x);//inverted???

        boolean pivotUp = (gamepad2.y);
        boolean pivotReset = (gamepad2.a);
        boolean pivotClimb = (gamepad2.x);

        boolean closeGrip = (gamepad2.left_bumper);
        boolean openGrip = (gamepad2.right_bumper);

        boolean spinClimber = (gamepad2.b);

        boolean resetElevatorPivotButton = (gamepad2.left_stick_button);

        boolean openPurplePixelGripper = (gamepad2.right_stick_button);
        boolean launchDrone = (gamepad2.right_stick_button);

     //   boolean  = (gamepad2.left_stick_y);
     //   boolean  = (gamepad2.right_stick_button);

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
        if(pivotUp && !UpPos) {
            UpPos = true;
            elevatorPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            moveToPos(elevatorPivotUpSpeed, pivotTarget);
            // May need to add a timeout here as it appears to take a while to stop.
        }

        // Move down to intake position
        if(pivotReset ) {
            UpPos = false;
            elevatorPivot.setPower(0);
            elevatorPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Move up to climbing position
        if (pivotClimb ) {
            elevatorPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            moveToPos(elevatorPivotClimbSpeed, pivotClimbTarget);
        }

        if(!climbHoist.isBusy()){
            climbHoist.setPower(0);
        }
        if(climbHoist.isBusy()) {
            elevatorPivot.setPower(0);
        }

        // Debugging to tell when the moveToPos function is complete
        if (elevatorPivot.isBusy()) {
            telemetry.addData("Still Moving - Current Pivot Motor Encoder Value ", elevatorPivot.getCurrentPosition());
        }
        // This is no longer requried as the power is set to 0 at time of lowering.
        //else if (!elevatorPivot.isBusy() && elevatorPivot.getCurrentPosition() == pivotHome){
        //    elevatorPivot.setPower(0);
        //}

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

          // Climber Control System
          if (spinClimber == true) {
            moveClimber(0.75);
          } else if (spinClimber == false) {
            moveClimber(0);
          }

          if (resetElevatorPivotButton == true) {
              elevatorPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
          }

          if(openPurplePixelGripper == true) {
              purplePixelGripper.setPosition(0.1);
          }

        if(launchDrone == true) {
            droneLauncher.setPosition(0.1);
        }
    }
}
