/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;



/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="2024: Auto Red Front", group="Robot", preselectTeleOp="2024 Teleop")
 public class AutoRedFront extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;
    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;


    private final int pivotHome = 0;
    private final int pivotTarget = -95;
    private boolean objectFound = false;
    enum Location{First,Second, Third}
    private Location objectLocation = null;
    private ColorSensor color;



    // Declare Motors.  All but climbing motor is required in auto
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor rearLeft = null;
    private DcMotor rearRight = null;
    private DcMotor elevatorPivot = null;

    // Declare Servos
    private Servo purplePixelGripper = null;
    private Servo gripperLeft;
    private Servo gripperRight;
    //  Declare beam breaker sensors.  They function the same as touch sensors so using the touch sensor class
    private TouchSensor LeftBeam = null;
    private TouchSensor RightBeam = null;
    private TouchSensor RearBeam = null;
    // runtime used for timeout during moves in case an obstetrical is encountered.
    private ElapsedTime     runtime = new ElapsedTime();


    // Counts per inch are based on field measurements

    static final double     DRIVE_COUNTS_PER_INCH         = 51;
    static final double     STRAFE_COUNTS_PER_INCH        = 51;
    static final double     DRIVE_SPEED             = 0.5;
    static final double     SEARCH_SPEED = 0.3;  // Just in case we need to reduce the speed when searching for an object.
    static final double     TURN_SPEED              = 0.5; //Not planning on peforming any turns in auto
    private final double maxSpeed = 0.625;   // Don't think this will be needed.
    static final double     CENTER_GRIPPER_OPEN = 0.1;
    private final double elevatorPivotUpSpeed = 1;  // Full power to lift
    private final double DISTANCE_TO_CENTER = 27.0;
    private final double DISTANCE_TO_BACKDROP = 30.0;

    private long TIME_SLEPT_AFTER_DROP = 50; //changes how long the robot sleeps for after dropping the purple pixel
    //important because we don't want the robot to start moving too early

    @Override
    public void runOpMode() {
        int TagTarget = 0;
        double DriveMove;
        double StrafeMove;

        RightBeam =  hardwareMap.get(TouchSensor.class, "Right");
        LeftBeam = hardwareMap.get(TouchSensor.class,"Left");
        RearBeam = hardwareMap.get(TouchSensor.class, "Rear");

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        rearRight = hardwareMap.get(DcMotor.class, "rearRight");

        elevatorPivot = hardwareMap.get(DcMotor.class, "elevatorPivot");


        purplePixelGripper = hardwareMap.get(Servo.class, "purplePixelGripper");
        gripperLeft = hardwareMap.get(Servo.class, "gripperLeft");
        gripperRight = hardwareMap.get(Servo.class, "gripperRight");

        color = hardwareMap.get(ColorSensor.class, "Color");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.FORWARD);
        purplePixelGripper.setDirection(Servo.Direction.REVERSE);
        gripperLeft.setDirection(Servo.Direction.REVERSE);
        gripperRight.setDirection(Servo.Direction.FORWARD);


        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        purplePixelGripper.setPosition(0);

        gripperLeft.setPosition(0);
        gripperRight.setPosition(0);



        /*        elevatorPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorPivot.setPower(elevatorPivotUpSpeed);
        elevatorPivot.setTargetPosition(-10);
        elevatorPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
*/
        // Wait for the game to start (driver presses PLAY)

        initAprilTag();

        waitForStart();

        encoderStrafe(DRIVE_SPEED, -30, 5);
        if (encoderStrafe(SEARCH_SPEED, -8, 5)){  // move robot to center on back line ready to drop purple pixel.  encoderStrafe will return true if object is encountered.
//            purplePixelGripper.setPosition(CENTER_GRIPPER_OPEN);  //  WORK Need to confirm proper operation of this servo and what direction is needed to drop the pixel.
            FindRedLineStrafe();
            objectFound = true;
            objectLocation = Location.Second;
            encoderStrafe(DRIVE_SPEED,-15,5);
            encoderDrive(DRIVE_SPEED,2.5*24,10); //inv
            encoderStrafe(DRIVE_SPEED,24,5);
        }
// Move back to center position
        if (!objectFound){
            encoderStrafe(DRIVE_SPEED,8,5);
            if(encoderDrive(SEARCH_SPEED,-5,5)){ //inv
                FindRedLineDrive();
                objectFound = true;
                objectLocation = Location.Third;
                encoderStrafe(DRIVE_SPEED,-26,5);
                encoderDrive(DRIVE_SPEED,3*24,10); //inv
                encoderStrafe(DRIVE_SPEED,24,5);
            }
        }
        // Move back to center position
        if (!objectFound){
            encoderDrive(DRIVE_SPEED, 17, 5); //inv
            FindRedLineDrive();
            objectFound = true;
            objectLocation = Location.First;
            encoderDrive(DRIVE_SPEED, 10, 5); //inv
            encoderStrafe(DRIVE_SPEED, -26, 5);
            encoderDrive(DRIVE_SPEED, 2*22, 5); //inv
            encoderStrafe(DRIVE_SPEED, 26, 5);
        }
        switch (objectLocation){
            case First:
                TagTarget = 4;
                break;
            case Second:
                TagTarget = 5;
                break;
            case Third:
                TagTarget = 6;
                break;
            default:
                TagTarget = 5;
                break;
        }

        elevatorPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorPivot.setPower(elevatorPivotUpSpeed);
        elevatorPivot.setTargetPosition(pivotTarget);
        elevatorPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //sleep(500);

        MoveToAprilTag(TagTarget);

        encoderDrive(SEARCH_SPEED,2,5);

        elevatorPivot.setPower(0);
        elevatorPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //drop pixel
        gripperLeft.setPosition(0.25);
        gripperRight.setPosition(0.25);
//        telemetry.addData("grip opening", gripperRight.getPosition());
//        telemetry.update();
        // move back

        encoderDrive(SEARCH_SPEED,-5,5);


        encoderStrafe(DRIVE_SPEED,-1*(20 + (TagTarget-4)*4),5);


//        encoderDrive(DRIVE_SPEED,5,5);


//        telemetry.addData("Path", "Complete");
//        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */

    public boolean MoveToAprilTag(int TagLocation){

        double drive = 0.0;
        double strafe = 0.0;
        boolean notInPosition = true;
        double maxSpeed = 0.2;

        double targetbearing = 0.0;
        double targetrange = 12.0;
        double currentRange = 0.0;

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        runtime.reset();
        while (notInPosition && opModeIsActive()) {

            boolean TargetTagFound = false;
            boolean TagFound = false;

            drive = 0.0;
            strafe = 0.0;

            List<AprilTagDetection> currentDetections = aprilTag.getDetections();

            telemetry.addLine(String.valueOf(currentDetections.size()));

            if (currentDetections.size() > 0) {

                for (AprilTagDetection detection : currentDetections) {
                    if (detection.id == TagLocation) {
                        runtime.reset();
                        TagFound = true;
                        TargetTagFound = true;
                        telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                        telemetry.addLine(String.format("\n==== %f range %f bearing", detection.ftcPose.range, detection.ftcPose.bearing));
                        telemetry.update();
                        if (Math.abs(detection.ftcPose.bearing) > 5) {
                            strafe = -1.0 * Math.signum(detection.ftcPose.bearing);
                            drive = 0.0;
                        } else {
                            drive = 1.0;
                            strafe = 0.0;
                        }
                        currentRange = detection.ftcPose.range;
/*
                    drive = Math.cos(detection.ftcPose.bearing / 180 * 3.1415);
                    strafe = -1 * Math.sin(detection.ftcPose.bearing / 180 *3.1415);

 */
                    } else if (TargetTagFound == false) {
                        drive = 0.0;
                        strafe = 0.0;

                        switch (detection.id) {
                            case 4:
                                break;
                            case 5:
                                break;
                            case 6:
                                break;
                        }
                    }
                }   // end for() loop
            }
            frontLeft.setPower(maxSpeed*(drive - strafe ));
            frontRight.setPower(maxSpeed*(drive - strafe ));
            rearLeft.setPower(maxSpeed*(drive + strafe ));
            rearRight.setPower(maxSpeed*(drive + strafe ));

            if ((currentRange < targetrange) || (runtime.seconds() > 3.0)){
                frontLeft.setPower(0.0);
                frontRight.setPower(0.0);
                rearLeft.setPower(0.0);
                rearRight.setPower(0.0);
                notInPosition = false;
            }

            sleep(10);
        }
        return true;
    }
    public void FindRedLineStrafe (){

        final int interval = 5;
        final int NPoints = 100;
        final double RunPower = 0.1;
        final double SearchPower = 0.07;

        int[] search = new int[NPoints];
        int CurrPos = 0;



        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() - interval*(NPoints/2));
        frontRight.setTargetPosition(frontRight.getCurrentPosition() - interval*(NPoints/2));
        rearRight.setTargetPosition(rearRight.getCurrentPosition() + interval*(NPoints/2));
        rearLeft.setTargetPosition(rearLeft.getCurrentPosition() + interval*(NPoints/2));

        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(RunPower);
        frontRight.setPower(RunPower);
        rearLeft.setPower(RunPower);
        rearRight.setPower(RunPower);

        while (opModeIsActive() && (frontLeft.isBusy() || frontRight.isBusy()) || rearLeft.isBusy() || rearLeft.isBusy()) {
            telemetry.addData("Move Operation","Complete");
        }

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setPower(SearchPower);
        frontRight.setPower(SearchPower);
        rearLeft.setPower(-1*SearchPower);
        rearRight.setPower(-1*SearchPower);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        for (int i=0;(i<NPoints) && opModeIsActive();i++) {
            while (((CurrPos = frontLeft.getCurrentPosition()) < i*interval)  && opModeIsActive());
//             RobotLog.d("Color: "+ Integer.toString(CurrPos) +","+ Integer.toString(redsearch[i] = color.red())+","+Integer.toString(color.green())+","+Integer.toString(color.blue())+","+Integer.toString(color.alpha()));
            search[i] = color.red();
        }

        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        rearLeft.setPower(0.0);
        rearRight.setPower(0.0);

        int maxred = 0;
        int maxloc = 0;
        for (int i = 0; i< 100; i++){
            if( search[i] > maxred){
                maxred = search[i];
                maxloc = i*interval;
            }
        }

        frontLeft.setTargetPosition(maxloc);
        frontRight.setTargetPosition(maxloc);
        rearRight.setTargetPosition(-1*maxloc);
        rearLeft.setTargetPosition(-1*maxloc);

        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        frontLeft.setPower(RunPower);
        frontRight.setPower(RunPower);
        rearLeft.setPower(RunPower);
        rearRight.setPower(RunPower);

        while (opModeIsActive() && (frontLeft.isBusy() || frontRight.isBusy()) || rearLeft.isBusy() || rearLeft.isBusy()) {
            telemetry.addData("Move Operation","Complete");
        }
        //sleep(100);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);

        purplePixelGripper.setPosition(CENTER_GRIPPER_OPEN);  //  WORK Need to confirm proper operation of this servo and what direction is needed to drop the pixel.
        sleep(TIME_SLEPT_AFTER_DROP); //second sleep function to ensure that the pixel drops before moving again
        /*        sleep(100);

        frontLeft.setTargetPosition(interval*NPoints);
        frontRight.setTargetPosition(interval*NPoints);
        rearRight.setTargetPosition(interval*NPoints);
        rearLeft.setTargetPosition(interval*NPoints);

        while (opModeIsActive() && (frontLeft.isBusy() || frontRight.isBusy()) || rearLeft.isBusy() || rearLeft.isBusy()) {
            telemetry.addData("Move Operation","Complete");
        }
*/
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void FindRedLineDrive (){

        final int interval = 5;
        final int NPoints = 100;
        final double RunPower = 0.3;
        final double SearchPower = 0.1;

        int[] search = new int[NPoints];
        int CurrPos = 0;



        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() - interval*(NPoints/2));
        frontRight.setTargetPosition(frontRight.getCurrentPosition() - interval*(NPoints/2));
        rearRight.setTargetPosition(rearRight.getCurrentPosition() - interval*(NPoints/2));
        rearLeft.setTargetPosition(rearLeft.getCurrentPosition() - interval*(NPoints/2));

        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(RunPower);
        frontRight.setPower(RunPower);
        rearLeft.setPower(RunPower);
        rearRight.setPower(RunPower);

        while (opModeIsActive() && (frontLeft.isBusy() || frontRight.isBusy()) || rearLeft.isBusy() || rearLeft.isBusy()) {
            telemetry.addData("Move Operation","Complete");
        }

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setPower(SearchPower);
        frontRight.setPower(SearchPower);
        rearLeft.setPower(SearchPower);
        rearRight.setPower(SearchPower);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        for (int i=0;(i<NPoints) && opModeIsActive();i++) {
            while (((CurrPos = frontLeft.getCurrentPosition()) < i*interval)  && opModeIsActive());
//             RobotLog.d("Color: "+ Integer.toString(CurrPos) +","+ Integer.toString(redsearch[i] = color.red())+","+Integer.toString(color.green())+","+Integer.toString(color.blue())+","+Integer.toString(color.alpha()));
            search[i] = color.red();
        }

        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        rearLeft.setPower(0.0);
        rearRight.setPower(0.0);

        int maxred = 0;
        int maxloc = 0;
        for (int i = 0; i< 100; i++){
            if( search[i] > maxred){
                maxred = search[i];
                maxloc = i*interval;
            }
        }
        frontLeft.setTargetPosition(maxloc);
        frontRight.setTargetPosition(maxloc);
        rearRight.setTargetPosition(maxloc);
        rearLeft.setTargetPosition(maxloc);

        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        frontLeft.setPower(RunPower);
        frontRight.setPower(RunPower);
        rearLeft.setPower(RunPower);
        rearRight.setPower(RunPower);

        while (opModeIsActive() && (frontLeft.isBusy() || frontRight.isBusy()) || rearLeft.isBusy() || rearLeft.isBusy()) {
            telemetry.addData("Move Operation","Complete");
        }
        //sleep(100);
        purplePixelGripper.setPosition(CENTER_GRIPPER_OPEN);  //  WORK Need to confirm proper operation of this servo and what direction is needed to drop the pixel.
        sleep(TIME_SLEPT_AFTER_DROP);
/*
        frontLeft.setTargetPosition(interval*NPoints);
        frontRight.setTargetPosition(interval*NPoints);
        rearRight.setTargetPosition(interval*NPoints);
        rearLeft.setTargetPosition(interval*NPoints);

        while (opModeIsActive() && (frontLeft.isBusy() || frontRight.isBusy()) || rearLeft.isBusy() || rearLeft.isBusy()) {
            telemetry.addData("Move Operation","Complete");
        }
        */
        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public boolean encoderStrafe(double speed,
                                 double inches,
                                 double timeoutS){
        // Return value is True if item is found and false if not.
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newRearLeftTarget;
        int newRearRightTarget;
        boolean objectFlag = false;

        if (opModeIsActive()){
            newFrontLeftTarget = frontLeft.getCurrentPosition()-(int)(inches * STRAFE_COUNTS_PER_INCH);
            newFrontRightTarget = frontRight.getCurrentPosition()-(int)(inches * STRAFE_COUNTS_PER_INCH);
            newRearLeftTarget = rearLeft.getCurrentPosition()+(int)(inches * STRAFE_COUNTS_PER_INCH);
            newRearRightTarget = rearRight.getCurrentPosition()+(int)(inches * STRAFE_COUNTS_PER_INCH);

            frontLeft.setTargetPosition(newFrontLeftTarget);
            frontRight.setTargetPosition(newFrontRightTarget);
            rearLeft.setTargetPosition(newRearLeftTarget);
            rearRight.setTargetPosition(newRearRightTarget);

            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();

            frontLeft.setPower(speed);
            frontRight.setPower(speed);
            rearLeft.setPower(speed);
            rearRight.setPower(speed);

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (frontLeft.isBusy() || frontRight.isBusy()) || rearLeft.isBusy() || rearLeft.isBusy()) {
                if(RightBeam.isPressed() || LeftBeam.isPressed() || RearBeam.isPressed()){
                    objectFlag = true;
                    telemetry.addData("Move Operation","Object found!");
                }
                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d :%7d :%7d",
                        newFrontLeftTarget,  newFrontRightTarget,
                        newRearLeftTarget, newRearRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d :%7d :%7d",
                        frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(),
                        rearLeft.getCurrentPosition(), rearRight.getCurrentPosition());
                telemetry.update();
            }

            frontLeft.setPower(0);
            frontRight.setPower(0);
            rearLeft.setPower(0);
            rearRight.setPower(0);

            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        return objectFlag;

    }

    public boolean encoderDrive(double speed,
                                double inches,
                                double timeoutS) {   // function returns true if one of the beams encounters an object during a move false if not.
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newRearLeftTarget;
        int newRearRightTarget;
        boolean objectFlag = false;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = frontLeft.getCurrentPosition()+(int)(inches * DRIVE_COUNTS_PER_INCH);
            newFrontRightTarget = frontRight.getCurrentPosition()+(int)(inches * DRIVE_COUNTS_PER_INCH);
            newRearLeftTarget = rearLeft.getCurrentPosition()+(int)(inches * DRIVE_COUNTS_PER_INCH);
            newRearRightTarget = rearRight.getCurrentPosition()+(int)(inches * DRIVE_COUNTS_PER_INCH);

            frontLeft.setTargetPosition(newFrontLeftTarget);
            frontRight.setTargetPosition(newFrontRightTarget);
            rearLeft.setTargetPosition(newRearLeftTarget);
            rearRight.setTargetPosition(newRearRightTarget);

            // Turn On RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            frontLeft.setPower(Math.abs(speed));
            frontRight.setPower(Math.abs(speed));
            rearLeft.setPower(Math.abs(speed));
            rearRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (frontLeft.isBusy() || frontRight.isBusy()) || rearLeft.isBusy() || rearRight.isBusy()) {
                if(RightBeam.isPressed() || LeftBeam.isPressed() || RearBeam.isPressed()){
                    objectFlag = true;
                }

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newFrontLeftTarget,  newFrontRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        frontLeft.getCurrentPosition(), frontRight.getCurrentPosition());
                telemetry.update();
            }

            // This is some test code that should open the purplePixelGripper to a random, untested, arbitrary position (10 degrees) -- Zachary and Ryan
            // instead of droping the pixel here I've changed the code to return a boolean if the object is found.
//            while (opModeIsActive() && (runtime.seconds() < timeoutS) && !leftDrive.isBusy() && !rightDrive.isBusy()) {
//                openPurplePixelGripper(purplePixelGripper, 10);
//            }

            // Stop all motion;
            frontLeft.setPower(0);
            frontRight.setPower(0);
            rearLeft.setPower(0);
            rearRight.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//            sleep(250);   // optional pause after each move.  This can be handeled in the main code if needed.
        }
        return objectFlag;
    }
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()

}

