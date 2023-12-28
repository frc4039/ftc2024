package org.firstinspires.ftc.teamcode;


/*
Code is writen for the T Bot not the FTC Bot.  It moves the robot approximately 4 inches recoding data from
color sensor to the log.

Next step it to work on algorithm to locate the center of a color line.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.RobotLog;

@Autonomous(name="2024: Test Colour", group="Robot")



public class TestColorSensor extends LinearOpMode {
    private ColorSensor color;
    private DcMotor MotorLeft;
    private DcMotor MotorRight;

    @Override
    public void runOpMode() {
        color = hardwareMap.get(ColorSensor.class, "Color");
        MotorLeft = hardwareMap.get(DcMotor.class,"driveMotorLeft");
        MotorRight = hardwareMap.get(DcMotor.class,"driveMotorRight");

        MotorLeft.setDirection(DcMotor.Direction.FORWARD);
        MotorRight.setDirection(DcMotor.Direction.REVERSE);
        MotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        color.enableLed(true);

        RobotLog.d("Color: Position, Red, Blue, Green");
        RobotLog.d(Integer.toString(color.red())+","+Integer.toString(color.blue())+","+Integer.toString(color.green()));


/*
         while(opModeIsActive()){
            telemetry.addData("red ",color.red());
            telemetry.addData("blue", color.blue());
            telemetry.addData("green", color.green());
            telemetry.addData("alpha",color.alpha());
            telemetry.update();
        }

 */
         MotorRight.setPower(.2);
         MotorLeft.setPower(.2);

         int[] redsearch = new int[100];

         int interval = 10;
         int CurrPos = 0;
         int PrevPos = MotorLeft.getCurrentPosition() - interval;
         for (int i=0;(i<100) && opModeIsActive();i++) {
             while (((CurrPos = MotorLeft.getCurrentPosition()) < i*interval)  && opModeIsActive());
//             RobotLog.d("Color: "+ Integer.toString(CurrPos) +","+ Integer.toString(redsearch[i] = color.red())+","+Integer.toString(color.green())+","+Integer.toString(color.blue())+","+Integer.toString(color.alpha()));
             redsearch[i] = color.red();
             PrevPos = CurrPos;
        }

        MotorRight.setPower(0.0);
        MotorLeft.setPower(0.0);

         int maxred = 0;
         int maxloc = 50*interval;
         for (int i = 0; i< 100; i++){
             if( redsearch[i] > maxred){
                 maxred = redsearch[i];
                 maxloc = i*interval;
             }
         }

         MotorLeft.setTargetPosition(maxloc);
         MotorRight.setTargetPosition(maxloc);
         MotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         MotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         MotorLeft.setPower(.5);
         MotorRight.setPower(.5);
         while(MotorRight.isBusy()&&opModeIsActive());
    }

    }
