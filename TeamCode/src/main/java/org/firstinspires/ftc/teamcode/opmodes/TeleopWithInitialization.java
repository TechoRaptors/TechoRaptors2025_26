package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.RobotInitialization;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "202526TR Teleop with Initialization", group = "TeleOpMode")
public class TeleopWithInitialization extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        RobotInitialization robot = new RobotInitialization(hardwareMap,telemetry,new Pose2d(0, 0, 0));
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            // MecanumDrive drive = robot.;new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
            //IMU imu = hardwareMap.get(IMU.class, "imu");
            robot.initialize();
            Deadline gamepadRateLimit = new Deadline(500, TimeUnit.MILLISECONDS);
            waitForStart();

            if (isStopRequested()) return;

            while (opModeIsActive()) {


// Field Centric Code
                double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                double x = gamepad1.left_stick_x;
                double rx = gamepad1.right_stick_x;
                if(gamepad1.right_trigger > 0)
                {
                    y = -gamepad1.left_stick_y/2; // Remember, Y stick value is reversed
                    x = gamepad1.left_stick_x/2;
                    rx = gamepad1.right_stick_x/2;
                }

                // This button choice was made so that it is hard to hit on accident,
                // it can be freely changed based on preference.
                // The equivalent button is start on Xbox-style controllers.
                if (gamepad1.left_bumper) {
                    robot.lazyImu.get().resetYaw();
                    gamepadRateLimit.reset();
                }

                double botHeading = robot.lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                // Rotate the movement direction counter to the bot's rotation
                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                rotX = rotX * 1.1;  // Counteract imperfect strafing


                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                double frontLeftPower = (rotY + rotX + rx) / denominator;
                double backLeftPower = (rotY - rotX + rx) / denominator;
                double frontRightPower = (rotY - rotX - rx) / denominator;
                double backRightPower = (rotY + rotX - rx) / denominator;


                robot.leftFront.setPower(frontLeftPower);
                robot.leftBack.setPower(backLeftPower);
                robot.rightFront.setPower(frontRightPower);
                robot.rightBack.setPower(backRightPower);



                robot.updatePoseEstimate();
                telemetry.addData("x", robot.localizer.getPose().position.x);
                telemetry.addData("y",  robot.localizer.getPose().position.y);
                //   telemetry.addData("Slider Position", robot.sliderMech.getPosition())
                telemetry.update();

                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), robot.localizer.getPose());
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }

        } else {
            throw new RuntimeException();
        }
    }
}
