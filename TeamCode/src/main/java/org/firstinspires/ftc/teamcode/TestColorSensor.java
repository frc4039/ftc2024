package org.firstinspires.ftc.teamcode;

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
         MotorRight.setPower(.1);
         MotorLeft.setPower(.1);

         int interval = 5;
         int CurrPos = 0;
         int PrevPos = MotorLeft.getCurrentPosition() - interval;
         for (int i=0;(i<100) && opModeIsActive();i++) {
             while (((CurrPos = MotorLeft.getCurrentPosition()) < i*interval)  && opModeIsActive());
             RobotLog.d("Color: "+ Integer.toString(CurrPos) +","+ Integer.toString(color.red())+","+Integer.toString(color.green())+","+Integer.toString(color.blue())+","+Integer.toString(color.alpha()));
             PrevPos = CurrPos;
        }

         MotorRight.setPower(0.0);
         MotorLeft.setPower(0.0);
    }

    }
