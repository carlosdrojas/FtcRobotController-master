
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
            leftArmDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightArmDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            grabberDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Pose2d startPose = new Pose2d(-61.25, -33, 0);
            drive.setPoseEstimate(startPose);

            TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                    .forward(40)
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