package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;


public class Robot2024 {

    // global items

    enum RobotOps {OpenGrippers, CloseGrippers, ClosePurple, OpenPurple, RaiseArm, LowerArm}
    enum RobotMove {XMove,YMove, XMoveSlow, YMoveSlow, XDrop, YDrop, AlignPos1, AlignPos2, AlignPos3}

    //  Motor Objects
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor rearLeft;
    private DcMotor rearRight;
    private DcMotor elevatorPivot;
    //  Servo Objects
    private Servo purplePixelGripper;
    private Servo gripperLeft;
    private Servo gripperRight;
    //  Beam Breaker Objects  Using touch sensor class
    private TouchSensor LeftBeam = null;
    private TouchSensor RightBeam = null;
    private TouchSensor RearBeam = null;
    //  Colour Sensor
    private ColorSensor color;
    private LinearOpMode opMode;



    Robot2024 (LinearOpMode OpModeNew){

        opMode = OpModeNew;
        // Map Motor Objects
        frontLeft = opMode.hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = opMode.hardwareMap.get(DcMotor.class, "frontRight");
        rearLeft = opMode.hardwareMap.get(DcMotor.class, "rearLeft");
        rearRight = opMode.hardwareMap.get(DcMotor.class, "rearRight");
        elevatorPivot = opMode.hardwareMap.get(DcMotor.class, "elevatorPivot");

        // Configure Motor operation

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.FORWARD);
        elevatorPivot.setDirection((DcMotorSimple.Direction.FORWARD));

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        // Map Servos

        purplePixelGripper = opMode.hardwareMap.get(Servo.class, "purplePixelGripper");
        gripperLeft = opMode.hardwareMap.get(Servo.class, "gripperLeft");
        gripperRight = opMode.hardwareMap.get(Servo.class, "gripperRight");

        // Configure Servos

        gripperLeft.setDirection(Servo.Direction.REVERSE);
        gripperRight.setDirection(Servo.Direction.FORWARD);


        // Map Beam Breakers

        RightBeam =  opMode.hardwareMap.get(TouchSensor.class, "Right");
        LeftBeam = opMode.hardwareMap.get(TouchSensor.class,"Left");
        RearBeam = opMode.hardwareMap.get(TouchSensor.class, "Rear");

        // Map Colour Sensor

        color = opMode.hardwareMap.get(ColorSensor.class,"Color");

        // Actions to take after INIT  close all three grippers

        Operate(RobotOps.CloseGrippers);
        Operate(RobotOps.ClosePurple);

    }

    void Operate ( RobotOps Operation){
        switch (Operation){
            case OpenGrippers:
                // move both grippers to poen position
                break;
            case CloseGrippers:
                // move both grippers to closed position
                break;
            case OpenPurple:
                // move purpole pixel gripper to open position
                break;
            case ClosePurple:
                // move purple pixel gripper to closed position
                break;
            case RaiseArm:
                // move arm up
                break;
            case LowerArm:
                // move arm down
                break;
            default:
                break;
        }
    }

    boolean  Move(RobotMove MoveType, double distance){
        switch (MoveType){
            case XMove:
                // fast move distance
                break;
            case YMove:
                // fast move distance
                break;
            case XMoveSlow:
                // Slow move distance
                break;
            case YMoveSlow:
                // Slow move distance
                break;
            case XDrop:
                // search for line in y direction by moving in the x direction and drop purple pixel
                break;
            case YDrop:
                //search for line in x direction by moving in the y direction and drop purple pixel
                break;
            case AlignPos1:
                // move robot in x direction to align with april tag pos 1 return true on success
                break;
            case AlignPos2:
                // move robot in x direction to align with april tag pos 2 return true on success
                break;
            case AlignPos3:
                // move robot in x direction to align with april tag pos 3 return true on success
                break;
            default:
                break;
        }
        return false;
    }
}

