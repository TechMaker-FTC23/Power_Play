package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DcMotor elevatorMotor;
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevatorMotor = hardwareMap.get(DcMotor.class, "elevatormotor");
        elevatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );
            if(gamepad1.y && elevatorMotor.getCurrentPosition()<5) {
                elevatorMotor.setPower(-0.5);
                telemetry.addData("Elevador","subindo");
            }
            else if(gamepad1.x && elevatorMotor.getCurrentPosition()>-2000) {
                elevatorMotor.setPower(0.5);
                telemetry.addData("Elevador","Descendo");
            }
            else{
                elevatorMotor.setPower(0);
            }
            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("Encoder Elevador",elevatorMotor.getCurrentPosition());
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
