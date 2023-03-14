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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



@TeleOp(name="state_machine_driver", group="Iterative Opmode")
//@Disabled
public class statemachine_driver extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor leftArmDrive;
    private DcMotor rightArmDrive;
    private DcMotor grabberDrive;
    private DcMotor turretDrive;
    private Servo grab;

    public enum LiftState {
        LIFT_START,
        LIFT_EXTEND_TRAVEL,
        LIFT_EXTEND_LOW_LEFT,
        LIFT_EXTEND_MID_LEFT,
        LIFT_EXTEND_HIGH_LEFT,
        LIFT_EXTEND_LOW_RIGHT,
        LIFT_EXTEND_MID_RIGHT,
        LIFT_EXTEND_HIGH_RIGHT,
        LIFT_RETRACT,
        LIFT_RETRACT_TRAVEL,
        LIFT_SCORE_LOW_LEFT,
        LIFT_SCORE_MID_LEFT,
        LIFT_SCORE_HIGH_LEFT,
        LIFT_SCORE_LOW_RIGHT,
        LIFT_SCORE_MID_RIGHT,
        LIFT_SCORE_HIGH_RIGHT,
    };

    public enum DriveState {
        DRIVER_FORWARD,
        DRIVER_BACKWARD,
    };

    DriveState driveState = DriveState.DRIVER_FORWARD;

    LiftState liftState = LiftState.LIFT_START;

    ElapsedTime liftTimer = new ElapsedTime();

    final double SCORE_TIME = .6;

    final int LIFT_TRAVEL = 200;
    final int LIFT_BOTTOM = 0;
    final int LIFT_LOW = 1200;
    final int LIFT_LOW_SCORE = 900;
    final int LIFT_MID = 2000;
    final int LIFT_MID_SCORE = 1650;
    final int LIFT_HIGH = 2900;
    final int LIFT_HIGH_SCORE = 2550;
    final int LIFT_LEFT = -700;
    final int LIFT_RIGHT = 700;
    final int LIFT_CENTER = 0;

    int direction;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");


        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftBackMotor");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackMotor");
        leftArmDrive = hardwareMap.get(DcMotor.class, "leftArmMotor");
        rightArmDrive = hardwareMap.get(DcMotor.class, "rightArmMotor");
        grabberDrive = hardwareMap.get(DcMotor.class, "grabberMotor");
        turretDrive = hardwareMap.get(DcMotor.class, "turretMotor");
        grab = hardwareMap.get(Servo.class, "grabServo");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        //leftArmDrive.setDirection(DcMotor.Direction.REVERSE);
        rightArmDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        leftArmDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArmDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        grabberDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftArmDrive.setTargetPosition(LIFT_BOTTOM);
        rightArmDrive.setTargetPosition(LIFT_BOTTOM);
        turretDrive.setTargetPosition(LIFT_CENTER);
        grabberDrive.setTargetPosition(0);

        leftArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        grabberDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        liftTimer.reset();

        runtime.reset();
    }



    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    @Override
    public void loop() {

        leftArmDrive.setPower(1);
        rightArmDrive.setPower(1);
        turretDrive.setPower(1);
        grabberDrive.setPower(1);

        switch (liftState) {
            case LIFT_START:
                //wait for input
                if (gamepad2.right_bumper) {
                    //right bumper pressed, start extending
                    leftArmDrive.setTargetPosition(LIFT_TRAVEL);
                    rightArmDrive.setTargetPosition(LIFT_TRAVEL);
                    liftState = LiftState.LIFT_EXTEND_TRAVEL;
                }
                if (gamepad2.dpad_left) {
                    //left dpad pressed, start extending
                    leftArmDrive.setTargetPosition(LIFT_LOW);
                    rightArmDrive.setTargetPosition(LIFT_LOW);
                    liftState = LiftState.LIFT_EXTEND_LOW_LEFT;
                }
                if (gamepad2.dpad_up) {
                    //up dpad pressed, start extending
                    leftArmDrive.setTargetPosition(LIFT_MID);
                    rightArmDrive.setTargetPosition(LIFT_MID);
                    liftState = LiftState.LIFT_EXTEND_MID_LEFT;
                }
                if (gamepad2.dpad_right) {
                    leftArmDrive.setTargetPosition(LIFT_HIGH);
                    rightArmDrive.setTargetPosition(LIFT_HIGH);
                    liftState = LiftState.LIFT_EXTEND_HIGH_LEFT;
                }
                if (gamepad2.x) {
                    leftArmDrive.setTargetPosition(LIFT_LOW);
                    rightArmDrive.setTargetPosition(LIFT_LOW);
                    liftState = LiftState.LIFT_EXTEND_LOW_RIGHT;
                }
                if (gamepad2.y) {
                    leftArmDrive.setTargetPosition(LIFT_MID);
                    rightArmDrive.setTargetPosition(LIFT_MID);
                    liftState = LiftState.LIFT_EXTEND_MID_RIGHT;
                }
                if (gamepad2.b) {
                    leftArmDrive.setTargetPosition(LIFT_HIGH);
                    rightArmDrive.setTargetPosition(LIFT_HIGH);
                    liftState = LiftState.LIFT_EXTEND_HIGH_RIGHT;
                }
                break;

            case LIFT_EXTEND_TRAVEL:
                //check if lift has extended to travel, otherwise do nothing
                if ((Math.abs(leftArmDrive.getCurrentPosition() - LIFT_TRAVEL) < 10) && (Math.abs(rightArmDrive.getCurrentPosition() - LIFT_TRAVEL) < 10) && gamepad2.a) {
                    leftArmDrive.setTargetPosition(LIFT_BOTTOM);
                    rightArmDrive.setTargetPosition(LIFT_BOTTOM);
                    liftState = LiftState.LIFT_RETRACT_TRAVEL;
                }
                if (gamepad2.dpad_left) {
                    //left dpad pressed, start extending
                    leftArmDrive.setTargetPosition(LIFT_LOW);
                    rightArmDrive.setTargetPosition(LIFT_LOW);
                    liftState = LiftState.LIFT_EXTEND_LOW_LEFT;
                }
                if (gamepad2.dpad_up) {
                    //up dpad pressed, start extending
                    leftArmDrive.setTargetPosition(LIFT_MID);
                    rightArmDrive.setTargetPosition(LIFT_MID);
                    liftState = LiftState.LIFT_EXTEND_MID_LEFT;
                }
                if (gamepad2.dpad_right) {
                    leftArmDrive.setTargetPosition(LIFT_HIGH);
                    rightArmDrive.setTargetPosition(LIFT_HIGH);
                    liftState = LiftState.LIFT_EXTEND_HIGH_LEFT;
                }
                if (gamepad2.x) {
                    leftArmDrive.setTargetPosition(LIFT_LOW);
                    rightArmDrive.setTargetPosition(LIFT_LOW);
                    liftState = LiftState.LIFT_EXTEND_LOW_RIGHT;
                }
                if (gamepad2.y) {
                    leftArmDrive.setTargetPosition(LIFT_MID);
                    rightArmDrive.setTargetPosition(LIFT_MID);
                    liftState = LiftState.LIFT_EXTEND_MID_RIGHT;
                }
                if (gamepad2.b) {
                    leftArmDrive.setTargetPosition(LIFT_HIGH);
                    rightArmDrive.setTargetPosition(LIFT_HIGH);
                    liftState = LiftState.LIFT_EXTEND_HIGH_RIGHT;
                }
            case LIFT_RETRACT_TRAVEL:
                if ((Math.abs(leftArmDrive.getCurrentPosition() - LIFT_BOTTOM) < 10) && (Math.abs(rightArmDrive.getCurrentPosition() - LIFT_BOTTOM) < 10)) {
                    liftState = LiftState.LIFT_START;
                }
                break;

            case LIFT_EXTEND_LOW_LEFT:
                if ((Math.abs(leftArmDrive.getCurrentPosition() - LIFT_LOW) < 20) && (Math.abs(rightArmDrive.getCurrentPosition() - LIFT_LOW) < 20)) {
                    turretDrive.setTargetPosition(LIFT_LEFT);
                }
                if (gamepad2.dpad_down) {
                    liftTimer.reset();
                    leftArmDrive.setTargetPosition(LIFT_LOW_SCORE);
                    rightArmDrive.setTargetPosition(LIFT_LOW_SCORE);
                    liftState = LiftState.LIFT_SCORE_LOW_LEFT;
                }
                if (gamepad2.a) {
                    leftArmDrive.setTargetPosition(LIFT_BOTTOM);
                    rightArmDrive.setTargetPosition(LIFT_BOTTOM);
                    turretDrive.setTargetPosition(LIFT_CENTER);
                    liftState = LiftState.LIFT_RETRACT;
                }
                break;

            case LIFT_SCORE_LOW_LEFT:
                if (liftTimer.seconds() >= SCORE_TIME) {
                    leftArmDrive.setTargetPosition(LIFT_LOW);
                    rightArmDrive.setTargetPosition(LIFT_LOW);
                    liftState = LiftState.LIFT_EXTEND_LOW_LEFT;
                }
                break;
            case LIFT_RETRACT:
                if ((Math.abs(leftArmDrive.getCurrentPosition() - LIFT_BOTTOM) < 10) && (Math.abs(rightArmDrive.getCurrentPosition() - LIFT_BOTTOM) < 10) && (Math.abs(turretDrive.getCurrentPosition() - LIFT_CENTER) < 10)) {
                    liftState = LiftState.LIFT_START;
                }
                break;
            case LIFT_EXTEND_MID_LEFT:
                if ((Math.abs(leftArmDrive.getCurrentPosition() - LIFT_MID) < 20) && (Math.abs(rightArmDrive.getCurrentPosition() - LIFT_MID) < 20)) {
                    turretDrive.setTargetPosition(LIFT_LEFT);
                }
                if (gamepad2.dpad_down) {
                    liftTimer.reset();
                    leftArmDrive.setTargetPosition(LIFT_MID_SCORE);
                    rightArmDrive.setTargetPosition(LIFT_MID_SCORE);
                    liftState = LiftState.LIFT_SCORE_MID_LEFT;
                }
                if (gamepad2.a) {
                    leftArmDrive.setTargetPosition(LIFT_BOTTOM);
                    rightArmDrive.setTargetPosition(LIFT_BOTTOM);
                    turretDrive.setTargetPosition(LIFT_CENTER);
                    liftState = LiftState.LIFT_RETRACT;
                }
                break;
            case LIFT_SCORE_MID_LEFT:
                if (liftTimer.seconds() >= SCORE_TIME) {
                    leftArmDrive.setTargetPosition(LIFT_MID);
                    rightArmDrive.setTargetPosition(LIFT_MID);
                    liftState = LiftState.LIFT_EXTEND_MID_LEFT;
                }
                break;
            case LIFT_EXTEND_HIGH_LEFT:
                if ((Math.abs(leftArmDrive.getCurrentPosition() - LIFT_HIGH) < 20) && (Math.abs(rightArmDrive.getCurrentPosition() - LIFT_HIGH) < 20)) {
                    turretDrive.setTargetPosition(LIFT_LEFT);
                }
                if (gamepad2.dpad_down) {
                    liftTimer.reset();
                    leftArmDrive.setTargetPosition(LIFT_HIGH_SCORE);
                    rightArmDrive.setTargetPosition(LIFT_HIGH_SCORE);
                    liftState = LiftState.LIFT_SCORE_HIGH_LEFT;
                }
                if (gamepad2.a) {
                    leftArmDrive.setTargetPosition(LIFT_BOTTOM);
                    rightArmDrive.setTargetPosition(LIFT_BOTTOM);
                    turretDrive.setTargetPosition(LIFT_CENTER);
                    liftState = LiftState.LIFT_RETRACT;
                }
                break;
            case LIFT_SCORE_HIGH_LEFT:
                if (liftTimer.seconds() >= SCORE_TIME) {
                    leftArmDrive.setTargetPosition(LIFT_HIGH);
                    rightArmDrive.setTargetPosition(LIFT_HIGH);
                    liftState = LiftState.LIFT_EXTEND_HIGH_LEFT;
                }
                break;
            case LIFT_EXTEND_LOW_RIGHT:
                if ((Math.abs(leftArmDrive.getCurrentPosition() - LIFT_LOW) < 20) && (Math.abs(rightArmDrive.getCurrentPosition() - LIFT_LOW) < 20)) {
                    turretDrive.setTargetPosition(LIFT_RIGHT);
                }
                if (gamepad2.dpad_down) {
                    liftTimer.reset();
                    leftArmDrive.setTargetPosition(LIFT_LOW_SCORE);
                    rightArmDrive.setTargetPosition(LIFT_LOW_SCORE);
                    liftState = LiftState.LIFT_SCORE_LOW_RIGHT;
                }
                if (gamepad2.a) {
                    leftArmDrive.setTargetPosition(LIFT_BOTTOM);
                    rightArmDrive.setTargetPosition(LIFT_BOTTOM);
                    turretDrive.setTargetPosition(LIFT_CENTER);
                    liftState = LiftState.LIFT_RETRACT;
                }
                break;
            case LIFT_SCORE_LOW_RIGHT:
                if (liftTimer.seconds() >= SCORE_TIME) {
                    leftArmDrive.setTargetPosition(LIFT_LOW);
                    rightArmDrive.setTargetPosition(LIFT_LOW);
                    liftState = LiftState.LIFT_EXTEND_LOW_RIGHT;
                }
                break;
            case LIFT_EXTEND_MID_RIGHT:
                if ((Math.abs(leftArmDrive.getCurrentPosition() - LIFT_MID) < 20) && (Math.abs(rightArmDrive.getCurrentPosition() - LIFT_MID) < 20)) {
                    turretDrive.setTargetPosition(LIFT_RIGHT);
                }
                if (gamepad2.dpad_down) {
                    liftTimer.reset();
                    leftArmDrive.setTargetPosition(LIFT_MID_SCORE);
                    rightArmDrive.setTargetPosition(LIFT_MID_SCORE);
                    liftState = LiftState.LIFT_SCORE_MID_RIGHT;
                }
                if (gamepad2.a) {
                    leftArmDrive.setTargetPosition(LIFT_BOTTOM);
                    rightArmDrive.setTargetPosition(LIFT_BOTTOM);
                    turretDrive.setTargetPosition(LIFT_CENTER);
                    liftState = LiftState.LIFT_RETRACT;
                }
                break;
            case LIFT_SCORE_MID_RIGHT:
                if (liftTimer.seconds() >= SCORE_TIME) {
                    leftArmDrive.setTargetPosition(LIFT_MID);
                    rightArmDrive.setTargetPosition(LIFT_MID);
                    liftState = LiftState.LIFT_EXTEND_MID_RIGHT;
                }
                break;
            case LIFT_EXTEND_HIGH_RIGHT:
                if ((Math.abs(leftArmDrive.getCurrentPosition() - LIFT_HIGH) < 20) && (Math.abs(rightArmDrive.getCurrentPosition() - LIFT_HIGH) < 20)) {
                    turretDrive.setTargetPosition(LIFT_RIGHT);
                }
                if (gamepad2.dpad_down) {
                    liftTimer.reset();
                    leftArmDrive.setTargetPosition(LIFT_HIGH_SCORE);
                    rightArmDrive.setTargetPosition(LIFT_HIGH_SCORE);
                    liftState = LiftState.LIFT_SCORE_HIGH_RIGHT;
                }
                if (gamepad2.a) {
                    leftArmDrive.setTargetPosition(LIFT_BOTTOM);
                    rightArmDrive.setTargetPosition(LIFT_BOTTOM);
                    turretDrive.setTargetPosition(LIFT_CENTER);
                    liftState = LiftState.LIFT_RETRACT;
                }
                break;
            case LIFT_SCORE_HIGH_RIGHT:
                if (liftTimer.seconds() >= SCORE_TIME) {
                    leftArmDrive.setTargetPosition(LIFT_HIGH);
                    rightArmDrive.setTargetPosition(LIFT_HIGH);
                    liftState = LiftState.LIFT_EXTEND_HIGH_RIGHT;
                }
                break;
            default:
                //should never be reached, as liftState should never be null
                liftState = LiftState.LIFT_START;
        }

        switch (driveState) {
            case DRIVER_FORWARD:
                direction = 1;
                if (gamepad1.a) {
                    driveState = DriveState.DRIVER_BACKWARD;
                }
                break;
            case DRIVER_BACKWARD:
                direction = -1;
                if (gamepad1.b) {
                    driveState = DriveState.DRIVER_FORWARD;
                }
                break;
        }



        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial   = -gamepad1.left_stick_y * direction;  // Note: pushing stick forward gives negative value
        double lateral =  gamepad1.left_stick_x * direction;
        double yaw     =  (-(gamepad1.right_stick_x) * 0.9) ;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }


        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower * 0.8);
        rightFrontDrive.setPower(rightFrontPower * 0.8);
        leftBackDrive.setPower(leftBackPower * 0.8);
        rightBackDrive.setPower(rightBackPower * 0.8);

        if (gamepad1.right_bumper) {
            //grab cone
            grab.setPosition(0.35);
        }
        else if (gamepad1.left_bumper) {
            //let go cone
            grab.setPosition(0);
        }
        //

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
