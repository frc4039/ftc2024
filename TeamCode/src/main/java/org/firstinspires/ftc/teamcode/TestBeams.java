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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@Autonomous(name="2024: Test Beams", group="Robot")
public class TestBeams extends LinearOpMode {



    private final int pivotHome = 0;
    private final int pivotTarget = -95;
    private boolean objectFound = false;
    enum Location{First,Second, Third}
    private Location objectLocation = null;


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
    static final double     DRIVE_SPEED             = 0.4039;
    static final double     SEARCH_SPEED = 0.2;  // Just in case we need to reduce the speed when searching for an object.
    static final double     TURN_SPEED              = 0.5; //Not planning on peforming any turns in auto
    private final double maxSpeed = 0.625;   // Don't think this will be needed.
    static final double     CENTER_GRIPPER_OPEN = 0.1;
    private final double elevatorPivotUpSpeed = 1;  // Full power to lift

    @Override
     public void runOpMode() {
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
        gripperLeft.setPosition(0.0);
        gripperRight.setPosition(0.0);

        /*        elevatorPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorPivot.setPower(elevatorPivotUpSpeed);
        elevatorPivot.setTargetPosition(-10);
        elevatorPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
*/
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("LeftBeam", LeftBeam.isPressed());
            telemetry.addData("RightBeam", RightBeam.isPressed());
            telemetry.addData("RearBeam", RearBeam .isPressed());
            telemetry.update();
        }
        stop();


            encoderStrafe(DRIVE_SPEED, 28.0, 5);  // Move to center of second tile 36  - 8 inch 1 1/2 tiles - 1/2 robot width
        if (encoderStrafe(SEARCH_SPEED, 12.0,5)){  // move robot to center on back line ready to drop purple pixel.  encoderStrafe will return true if object is encountered.
            purplePixelGripper.setPosition(CENTER_GRIPPER_OPEN);  //  WORK Need to confirm proper operation of this servo and what direction is needed to drop the pixel.
            objectFound = true;
            objectLocation = Location.Second;
        }
        encoderStrafe(DRIVE_SPEED,-10,5);  // Move back to center position
        if (!objectFound){
            if(encoderDrive(SEARCH_SPEED,-12,5)){  // object found in position 2
                purplePixelGripper.setPosition(CENTER_GRIPPER_OPEN);
                objectFound = true;
                objectLocation = Location.Third;
                telemetry.addData("Detected","Third Position");
            }
            encoderDrive(DRIVE_SPEED,12,5);  // Move back to center position
        }
        if (!objectFound){
            encoderDrive(SEARCH_SPEED,6,5);
            purplePixelGripper.setPosition(CENTER_GRIPPER_OPEN);
            objectLocation = Location.First;
        }
        //raise arm
        elevatorPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorPivot.setPower(elevatorPivotUpSpeed);
        elevatorPivot.setTargetPosition(pivotTarget);
        elevatorPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        switch(objectLocation){
            case First:
                encoderDrive(DRIVE_SPEED,32,5);
                //drive 36
                break;
            default:
                encoderDrive(DRIVE_SPEED,42,5);
                break;
        }

        //drop pixel
        gripperLeft.setPosition(0.25);
        gripperRight.setPosition(0.25);
        telemetry.addData("grip opening", gripperRight.getPosition());
        telemetry.update();
        sleep(500);
        // move back
        encoderDrive(DRIVE_SPEED,-10,5);

        // lower arm
        elevatorPivot.setPower(0);
        elevatorPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(500);

        switch (objectLocation){
            case First:
                encoderStrafe(DRIVE_SPEED,-26,5);
                break;
            case Second:
                encoderStrafe(DRIVE_SPEED,-26,5);
                break;
            default:
                encoderStrafe(DRIVE_SPEED,-26,5);
                break;
        }
        encoderDrive(DRIVE_SPEED,10,5);


        telemetry.addData("Path", "Complete");
        telemetry.update();
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

}