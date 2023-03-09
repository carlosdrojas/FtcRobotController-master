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
        LIFT_RETRACT_TRAVEL,
        LIFT_RETRACT_LOW_LEFT,
        LIFT_RETRACT_MID_LEFT,
        LIFT_RETRACT_HIGH_LEFT,
        LIFT_RETRACT_LOW_RIGHT,
        LIFT_RETRACT_MID_RIGHT,
        LIFT_RETRACT_HIGH_RIGHT,
        LIFT_SCORE_LOW,
        LIFT_SCORE_MID,
        LIFT_SCORE_HIGH,
    };

    LiftState liftState = LiftState.LIFT_START;

    ElapsedTime liftTimer = new ElapsedTime();

    final double SCORE_TIME = 1;

    final int LIFT_TRAVEL = 200;
    final int LIFT_BOTTOM = 0;
    final int LIFT_LOW = 1200;
    final int LIFT_LOW_SCORE = 850;
    final int LIFT_MID = 2000;
    final int LIFT_MID_SCORE = 1650;
    final int LIFT_HIGH = 2900;
    final int LIFT_HIGH_SCORE = 2550;
    final int LIFT_LEFT = -700;
    final int LIFT_RIGHT = 700;
    final int LIFT_MIDDLE = 0;

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

        leftArmDrive.setTargetPosition(LIFT_BOTTOM);
        rightArmDrive.setTargetPosition(LIFT_BOTTOM);
        turretDrive.setTargetPosition(LIFT_MIDDLE);

        leftArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

        leftArmDrive.setPower(0.5);
        rightArmDrive.setPower(0.5);
        turretDrive.setPower(0.5);

        switch (liftState) {
            case LIFT_START:
                //wait for input
                if (gamepad1.right_bumper) {
                    //right bumper pressed, start extending
                    leftArmDrive.setTargetPosition(LIFT_TRAVEL);
                    rightArmDrive.setTargetPosition(LIFT_TRAVEL);
                    liftState = LiftState.LIFT_EXTEND_TRAVEL;
                }
                if (gamepad1.dpad_left) {
                    //left dpad pressed, start extending
                    leftArmDrive.setTargetPosition(LIFT_LOW);
                    rightArmDrive.setTargetPosition(LIFT_LOW);
                    liftState = LiftState.LIFT_EXTEND_LOW_LEFT;
                }
                break;
            case LIFT_EXTEND_TRAVEL:
                //check if lift has extended to travel, otherwise do nothing
                if ((Math.abs(leftArmDrive.getCurrentPosition() - LIFT_TRAVEL) < 10) && (Math.abs(rightArmDrive.getCurrentPosition() - LIFT_TRAVEL) < 10) && gamepad1.a) {
                    leftArmDrive.setTargetPosition(LIFT_BOTTOM);
                    rightArmDrive.setTargetPosition(LIFT_BOTTOM);
                    liftState = LiftState.LIFT_RETRACT_TRAVEL;
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
                    leftArmDrive.setTargetPosition(LIFT_LOW_SCORE);
                    rightArmDrive.setTargetPosition(LIFT_LOW_SCORE);
                    liftState = LiftState.LIFT_SCORE_LOW;
                }
                if (gamepad1.a) {
                    leftArmDrive.setTargetPosition(LIFT_BOTTOM);
                    rightArmDrive.setTargetPosition(LIFT_BOTTOM);
                    turretDrive.setTargetPosition(LIFT_MIDDLE);
                    liftState = LiftState.LIFT_RETRACT_LOW_LEFT;
                }
                break;
            case LIFT_SCORE_LOW:
                if (liftTimer.seconds() >= SCORE_TIME) {
                    leftArmDrive.setTargetPosition(LIFT_LOW);
                    rightArmDrive.setTargetPosition(LIFT_LOW);
                    liftState = LiftState.LIFT_EXTEND_LOW_LEFT;
                }

            case LIFT_RETRACT_LOW_LEFT:
                if ((Math.abs(leftArmDrive.getCurrentPosition() - LIFT_BOTTOM) < 10) && (Math.abs(rightArmDrive.getCurrentPosition() - LIFT_BOTTOM) < 10)) {
                    liftState = LiftState.LIFT_START;
                }
                break;
            default:
                //should never be reached, as liftState should never be null
                liftState = LiftState.LIFT_START;
        }



        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral =  gamepad1.left_stick_x;
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

        // This is test code:
        //
        // Uncomment the following code to test your motor directions.
        // Each button should make the corresponding motor run FORWARD.
        //   1) First get all the motors to take to correct positions on the robot
        //      by adjusting your Robot Configuration if necessary.
        //   2) Then make sure they run in the correct direction by modifying the
        //      the setDirection() calls above.
        // Once the correct motors move in the correct direction re-comment this code.
            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower * 0.7);
        rightFrontDrive.setPower(rightFrontPower * 0.7);
        leftBackDrive.setPower(leftBackPower * 0.7);
        rightBackDrive.setPower(rightBackPower * 0.7);

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
