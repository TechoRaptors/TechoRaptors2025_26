package org.firstinspires.ftc.teamcode.robot;


import com.acmerobotics.roadrunner.Pose2d;

import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class RobotInitialization extends MecanumDrive {

    HardwareMap robotHardwareMap ;
    public Telemetry localTelemetry ;
    //public GoBildaPinpointDriver odo;





    public RobotInitialization(HardwareMap hardwareMap, Telemetry pTelemetry, Pose2d pose2D) {
        super(hardwareMap, pose2D);
        robotHardwareMap = hardwareMap;
        localTelemetry = pTelemetry;


    }

    public void initialize(){

        localTelemetry.addData("Intialized","All parts");

    }
    public void initializeTeleop(){


    }

}
