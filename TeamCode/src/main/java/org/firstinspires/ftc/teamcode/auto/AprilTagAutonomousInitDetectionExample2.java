
package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;

//@Disabled
@Autonomous(name = "cracked_auto")
public class AprilTagAutonomousInitDetectionExample2 extends LinearOpMode
{
    private DcMotor leftArmDrive;
    private DcMotor rightArmDrive;
    private DcMotor turretDrive;
    private DcMotor grabberDrive;
    private Servo grab;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

     // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;



    @Override
    public void runOpMode() {

            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

            camera.setPipeline(aprilTagDetectionPipeline);
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {

                }
            });

            telemetry.setMsTransmissionInterval(50);



            /*
             * The INIT-loop:
             * This REPLACES waitForStart!
             */

            leftArmDrive = hardwareMap.get(DcMotor.class, "leftArmMotor");
            rightArmDrive = hardwareMap.get(DcMotor.class, "rightArmMotor");
            turretDrive = hardwareMap.get(DcMotor.class, "turretMotor");
            grabberDrive = hardwareMap.get(DcMotor.class, "grabberMotor");
            grab = hardwareMap.get(Servo.class, "grabServo");

            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

            /*
            leftArmDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightArmDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            grabberDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            turretDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

             */

            leftArmDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightArmDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            turretDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            grabberDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            rightArmDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            grabberDrive.setDirection(DcMotorSimple.Direction.REVERSE);

            Pose2d startPose = new Pose2d(-61.25, -25.25, 0);

            drive.setPoseEstimate(startPose);

            TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                    .waitSeconds(1)
                    .lineToConstantHeading(new Vector2d(-60, -29))
                    .forward(47)
                    .waitSeconds(2)
                    //GRAB CONE
                    .addTemporalMarker(0, () -> {
                        grab.setPosition(0.35);

                    })
                    //EXTEND ARM
                    .addTemporalMarker(1.3, () -> {
                        grabberDrive.setTargetPosition(1000);
                        grabberDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        grabberDrive.setPower(1);

                    })
                    //RAISE ARM SLIGHTLY
                    .addTemporalMarker(1.5, () -> {
                        leftArmDrive.setTargetPosition(200);
                        rightArmDrive.setTargetPosition(200);
                        leftArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftArmDrive.setPower(1);
                        rightArmDrive.setPower(1);

                    })
                    //TURN TURRET TO MIDDLE
                    .addTemporalMarker(1.5, () -> {
                        turretDrive.setTargetPosition(0);
                        turretDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        turretDrive.setPower(.6);

                    })
                    //RAISE ARM TO HIGH SCORE POS
                    .addTemporalMarker(2, () -> {
                        leftArmDrive.setTargetPosition(2900);
                        rightArmDrive.setTargetPosition(2900);
                        leftArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftArmDrive.setPower(.6);
                        rightArmDrive.setPower(.6);

                    })
                    //TURN TURRET PRE-SCORE POS
                    .addTemporalMarker(2.6, () -> {
                        turretDrive.setTargetPosition(100);
                        turretDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        turretDrive.setPower(.6);

                    })
                    //TURN TURRET SCORE POS
                    .addTemporalMarker(3.9, () -> {
                        turretDrive.setTargetPosition(400);
                        turretDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        turretDrive.setPower(.6);

                    })
                    //LOWER ARM SCORE POS
                    .addTemporalMarker(4.13, () -> {
                        leftArmDrive.setTargetPosition(2500);
                        rightArmDrive.setTargetPosition(2500);
                        leftArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftArmDrive.setPower(.6);
                        rightArmDrive.setPower(.6);

                    })
                    //LET GO CONE
                    .addTemporalMarker(4.7, () -> {
                        grab.setPosition(0);

                    })
                    //RETRACT ARM
                    .addTemporalMarker(5, () -> {
                        grabberDrive.setTargetPosition(700);
                        grabberDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        grabberDrive.setPower(1);

                    })
                    //TURN TURRET TO STACK
                    .addTemporalMarker(5.2, () -> {
                        turretDrive.setTargetPosition(-748);
                        turretDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        turretDrive.setPower(.6);
                    })
                    //LOWER TURRET TO STACK
                    .addTemporalMarker(5.4, () -> {
                        leftArmDrive.setTargetPosition(490);
                        rightArmDrive.setTargetPosition(490);
                        leftArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftArmDrive.setPower(.6);
                        rightArmDrive.setPower(.6);
                    })
                    //EXTEND ARM
                    .addTemporalMarker(5.7, () -> {
                        grabberDrive.setTargetPosition(1700);
                        grabberDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        grabberDrive.setPower(1);

                    })
                    //EXTEND ARM FULLY
                    .addTemporalMarker(7.6, () -> {
                        grabberDrive.setTargetPosition(2000);
                        grabberDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        grabberDrive.setPower(1);

                    })
                    //GRAB CONE 2
                    .addTemporalMarker(8, () -> {
                        grab.setPosition(0.35);

                    })
                    //RAISE ARM WITH GRAB CONE 2
                    .addTemporalMarker(8.5, () -> {
                        leftArmDrive.setTargetPosition(900);
                        rightArmDrive.setTargetPosition(900);
                        leftArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftArmDrive.setPower(.6);
                        rightArmDrive.setPower(.6);
                    })
                    //TURN TURRET TO SCORE POS 2
                    .addTemporalMarker(9.5, () -> {
                        turretDrive.setTargetPosition(500);
                        turretDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        turretDrive.setPower(.3);
                    })
                    //RAISE TURRET TO SCORE POS 2
                    .addTemporalMarker(9.6, () -> {
                        leftArmDrive.setTargetPosition(2900);
                        rightArmDrive.setTargetPosition(2900);
                        leftArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftArmDrive.setPower(1);
                        rightArmDrive.setPower(1);

                    })
                    //RETRACT ARM TO SCORE POS 2
                    .addTemporalMarker(10, () -> {
                        grabberDrive.setTargetPosition(1500);
                        grabberDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        grabberDrive.setPower(1);

                    })
                    //LOWER ARM TO DUNK SCORE POS 2
                    .addTemporalMarker(11.39, () -> {
                        leftArmDrive.setTargetPosition(2500);
                        rightArmDrive.setTargetPosition(2500);
                        leftArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftArmDrive.setPower(.8);
                        rightArmDrive.setPower(.8);

                    })
                    //LET GO CONE 2
                    .addTemporalMarker(12, () -> {
                        grab.setPosition(0);

                    })

                    .lineToLinearHeading(new Pose2d(-11, -40, 0 ))
                    .waitSeconds(6)
                    //STRAFE TO GRAB THRID CONE
                    .strafeRight(1)
                    //RETRACT ARM
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        grabberDrive.setTargetPosition(700);
                        grabberDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        grabberDrive.setPower(1);
                    })
                    //TURN TURRET TO GRAB 3 CONE
                    .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                        turretDrive.setTargetPosition(-745);
                        turretDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        turretDrive.setPower(.6);
                    })
                    //LOWER ARM TO GRAB 3 CONE
                    .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                        leftArmDrive.setTargetPosition(430);
                        rightArmDrive.setTargetPosition(430);
                        leftArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftArmDrive.setPower(.6);
                        rightArmDrive.setPower(.6);
                    })
                    //EXTEND ARM TO GRAB 3 CONE
                    .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                        grabberDrive.setTargetPosition(1700);
                        grabberDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        grabberDrive.setPower(1);
                    })
                    //FULLY EXTEND ARM TO GRAB 3 CONE
                    .UNSTABLE_addTemporalMarkerOffset(2.7, () -> {
                        grabberDrive.setTargetPosition(1990);
                        grabberDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        grabberDrive.setPower(1);
                    })
                    //GRAB 3 CONE
                    .UNSTABLE_addTemporalMarkerOffset(2.9, () -> {
                        grab.setPosition(0.35);

                    })
                    //LIFT ARM TO GRAB 3 CONE
                    .UNSTABLE_addTemporalMarkerOffset(3.6, () -> {
                        leftArmDrive.setTargetPosition(900);
                        rightArmDrive.setTargetPosition(900);
                        leftArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftArmDrive.setPower(.6);
                        rightArmDrive.setPower(.6);

                    })
                    //ROTATE TURRET TO 3 CONE SCORE POS
                    .UNSTABLE_addTemporalMarkerOffset(4.6, () -> {
                        turretDrive.setTargetPosition(500);
                        turretDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        turretDrive.setPower(.3);

                    })
                    //RAISE ARM TO 3 CONE SCORE POS
                    .UNSTABLE_addTemporalMarkerOffset(4.7, () -> {
                        leftArmDrive.setTargetPosition(2900);
                        rightArmDrive.setTargetPosition(2900);
                        leftArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftArmDrive.setPower(1);
                        rightArmDrive.setPower(1);

                    })
                    //RETRACT ARM
                    .UNSTABLE_addTemporalMarkerOffset(5.1, () -> {
                        grabberDrive.setTargetPosition(1500);
                        grabberDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        grabberDrive.setPower(1);

                    })
                    //LOWER ARM TO 3 CONE DUNK
                    .UNSTABLE_addTemporalMarkerOffset(6.35, () -> {
                        leftArmDrive.setTargetPosition(2500);
                        rightArmDrive.setTargetPosition(2500);
                        leftArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftArmDrive.setPower(.8);
                        rightArmDrive.setPower(.8);
                    })

                    .waitSeconds(6)
                    .strafeLeft(1)
                    .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                        grab.setPosition(0);
                    })
                    .waitSeconds(2)
                    .strafeRight(1)
                    //RETRACT ARM
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        grabberDrive.setTargetPosition(700);
                        grabberDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        grabberDrive.setPower(1);
                    })
                    //TURN TURRET TO GRAB 4 CONE
                    .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                        turretDrive.setTargetPosition(-740);
                        turretDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        turretDrive.setPower(.6);
                    })
                    //LOWER ARM TO GRAB 4 CONE
                    .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                        leftArmDrive.setTargetPosition(370);
                        rightArmDrive.setTargetPosition(370);
                        leftArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftArmDrive.setPower(.6);
                        rightArmDrive.setPower(.6);
                    })
                    //EXTEND ARM TO GRAB 4 CONE
                    .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                        grabberDrive.setTargetPosition(1700);
                        grabberDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        grabberDrive.setPower(1);
                    })
                    //FULLY EXTEND ARM TO GRAB 4 CONE
                    .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                        grabberDrive.setTargetPosition(1985);
                        grabberDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        grabberDrive.setPower(1);
                    })
                    //GRAB 4 CONE
                    .UNSTABLE_addTemporalMarkerOffset(2.5, () -> {
                        grab.setPosition(0.35);

                    })
                    //LIFT ARM TO GRAB 4 CONE
                    .UNSTABLE_addTemporalMarkerOffset(3.6, () -> {
                        leftArmDrive.setTargetPosition(900);
                        rightArmDrive.setTargetPosition(900);
                        leftArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftArmDrive.setPower(.6);
                        rightArmDrive.setPower(.6);

                    })
                    //ROTATE TURRET TO 4 CONE SCORE POS
                    .UNSTABLE_addTemporalMarkerOffset(4.6, () -> {
                        turretDrive.setTargetPosition(470);
                        turretDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        turretDrive.setPower(.3);

                    })
                    //RAISE ARM TO 4 CONE SCORE POS
                    .UNSTABLE_addTemporalMarkerOffset(4.7, () -> {
                        leftArmDrive.setTargetPosition(2900);
                        rightArmDrive.setTargetPosition(2900);
                        leftArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftArmDrive.setPower(1);
                        rightArmDrive.setPower(1);

                    })
                    //RETRACT ARM
                    .UNSTABLE_addTemporalMarkerOffset(5.1, () -> {
                        grabberDrive.setTargetPosition(1500);
                        grabberDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        grabberDrive.setPower(1);

                    })
                    //LOWER ARM TO 4 CONE DUNK
                    .UNSTABLE_addTemporalMarkerOffset(6.58, () -> {
                        leftArmDrive.setTargetPosition(2500);
                        rightArmDrive.setTargetPosition(2500);
                        leftArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftArmDrive.setPower(.8);
                        rightArmDrive.setPower(.8);
                    })
                    .waitSeconds(6)
                    .strafeLeft(1)
                    .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                        grab.setPosition(0);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                        leftArmDrive.setTargetPosition(0);
                        rightArmDrive.setTargetPosition(0);
                        leftArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftArmDrive.setPower(.8);
                        rightArmDrive.setPower(.8);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                        grabberDrive.setTargetPosition(0);
                        grabberDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        grabberDrive.setPower(1);                    })
                    .waitSeconds(0.8)
                    .build();

            TrajectorySequence left = drive.trajectorySequenceBuilder(traj1.end())
                    .strafeLeft(33)
                    .build();

            TrajectorySequence right = drive.trajectorySequenceBuilder(traj1.end())
                    .strafeRight(24)
                    .build();


            while (!isStarted() && !isStopRequested()) {
                ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

                if (currentDetections.size() != 0) {
                    boolean tagFound = false;

                    for (AprilTagDetection tag : currentDetections) {
                        if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                            tagOfInterest = tag;
                            tagFound = true;
                            break;
                        }
                    }

                    if (tagFound) {
                        telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                        tagToTelemetry(tagOfInterest);
                    } else {
                        telemetry.addLine("Don't see tag of interest :(");

                        if (tagOfInterest == null) {
                            telemetry.addLine("(The tag has never been seen)");
                        } else {
                            telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                            tagToTelemetry(tagOfInterest);
                        }
                    }

                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }

                }

                telemetry.update();
                sleep(20);
            }

            /*
             * The START command just came in: now work off the latest snapshot acquired
             * during the init loop.
             */

            /* Update the telemetry */
            if (tagOfInterest != null) {
                telemetry.addLine("Tag snapshot:\n");
                tagToTelemetry(tagOfInterest);
                telemetry.update();
            } else {
                telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
                telemetry.update();
            }

            /* Actually do something useful */
            if (tagOfInterest == null || tagOfInterest.id == LEFT) {
                //trajectory

                drive.followTrajectorySequence(traj1);
                drive.followTrajectorySequence(left);

            }

            else if (tagOfInterest.id == MIDDLE) {
                //trajectory

                drive.followTrajectorySequence(traj1);


            } else {
                //trajectory

                drive.followTrajectorySequence(traj1);
                drive.followTrajectorySequence(right);


            }


            /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
            //while (opModeIsActive()) {sleep(20);}

    }
    //this is where u put the bracket

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }


}