package org.firstinspires.ftc.teamcode.RoadRunnterTest;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous (name = "a_test")
public class test extends LinearOpMode {

    private DcMotor leftArmDrive;
    private DcMotor rightArmDrive;
    private DcMotor turretDrive;
    private DcMotor grabberDrive;
    private Servo grab;

    //private int armPos;
    //endocer stuff

    @Override
    public void runOpMode() {

        leftArmDrive = hardwareMap.get(DcMotor.class, "leftArmMotor");
        rightArmDrive = hardwareMap.get(DcMotor.class, "rightArmMotor");
        turretDrive = hardwareMap.get(DcMotor.class, "turretMotor");
        grabberDrive = hardwareMap.get(DcMotor.class, "grabberMotor");
        grab = hardwareMap.get(Servo.class, "grabServo");
        leftArmDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArmDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabberDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArmDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        grabberDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        /*
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        armPos = 0;
        //encoder stuff

         */



        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-61.25, -21.25, 0);

        drive.setPoseEstimate(startPose);

        //random line
        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                    .waitSeconds(1)
                    .lineToConstantHeading(new Vector2d(-60, -28.75))
                    .lineToConstantHeading(new Vector2d(-22, -28.75))
                    // .forward(38)
                    .waitSeconds(1.5)
                    //GRAB CONE
                    .addTemporalMarker(0, () -> {
                        grab.setPosition(125);

                    })
                    //RAISE ARM SLIGHTLY
                    .addTemporalMarker(1.5, () -> {
                        leftArmDrive.setTargetPosition(2200);
                        rightArmDrive.setTargetPosition(2200);
                        leftArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftArmDrive.setPower(1);
                        rightArmDrive.setPower(1);

                    })
                    //dunk arm cone 1 med
                    .addTemporalMarker(3.9, () -> {
                        leftArmDrive.setTargetPosition(1650);
                        rightArmDrive.setTargetPosition(1650);
                        leftArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftArmDrive.setPower(1);
                        rightArmDrive.setPower(1);

                    })
                    //let go cone 1
                    .addTemporalMarker(4.0, () -> {
                        grab.setPosition(0);

                    })
                    //TURN TURRET TO STACK
                    .addTemporalMarker(4.9, () -> {
                        turretDrive.setTargetPosition(750);
                        turretDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        turretDrive.setPower(.6);
                    })
                    .addTemporalMarker(4.7, () -> {
                        leftArmDrive.setTargetPosition(490);
                        rightArmDrive.setTargetPosition(490);
                        leftArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftArmDrive.setPower(.6);
                        rightArmDrive.setPower(.6);

                    })
                    //move forward little
                    .lineToConstantHeading(new Vector2d(-10, -28.75))
                    //move to stack
                    .lineToLinearHeading(new Pose2d(-11, -14, Math.toRadians(91)))
                    .waitSeconds(10)
                    //extend arm to score pos
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        grabberDrive.setTargetPosition(1300);
                        grabberDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        grabberDrive.setPower(.8);

                    })
                    //grab cone 2
                    .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                        grab.setPosition(125);

                    })
                    //rotate to score 2
                    .UNSTABLE_addTemporalMarkerOffset(2.5, () -> {
                        leftArmDrive.setTargetPosition(1300);
                        rightArmDrive.setTargetPosition(1300);
                        leftArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftArmDrive.setPower(.6);
                        rightArmDrive.setPower(.6);
                    })
                    //retract arm to score 2
                    .UNSTABLE_addTemporalMarkerOffset(3, () -> {
                        grabberDrive.setTargetPosition(150);
                        grabberDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        grabberDrive.setPower(.8);

                    })
                    //turn turret to score 2
                    .UNSTABLE_addTemporalMarkerOffset(3.2, () -> {
                        turretDrive.setTargetPosition(1500);
                        turretDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        turretDrive.setPower(.6);
                    })
                    //dunk 2
                    .UNSTABLE_addTemporalMarkerOffset(4, () -> {
                        leftArmDrive.setTargetPosition(900);
                        rightArmDrive.setTargetPosition(900);
                        leftArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftArmDrive.setPower(.6);
                        rightArmDrive.setPower(.6);
                    })
                    //let go cone 2
                    .UNSTABLE_addTemporalMarkerOffset(4.2, () -> {
                        grab.setPosition(0);

                    })
                    //reset arm ext pos
                    .UNSTABLE_addTemporalMarkerOffset(4.5, () -> {
                        grabberDrive.setTargetPosition(0);
                        grabberDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        grabberDrive.setPower(.8);

                    })

                    .UNSTABLE_addTemporalMarkerOffset(4.6, () -> {
                        leftArmDrive.setTargetPosition(400);
                        rightArmDrive.setTargetPosition(400);
                        leftArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftArmDrive.setPower(.6);
                        rightArmDrive.setPower(.6);

                    })
                    //reset stack pos
                    .UNSTABLE_addTemporalMarkerOffset(4.7, () -> {
                        turretDrive.setTargetPosition(750);
                        turretDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        turretDrive.setPower(.6);
                    })


                    //actually turning 750
                    /*
                    .addTemporalMarker(5, () -> {
                        turretDrive.setTargetPosition(-700);

                        turretDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        turretDrive.setPower(0.5);

                    })

                     */
                    .waitSeconds(30)
                    .build();
            drive.followTrajectorySequence(trajSeq);
        }
    }


}
