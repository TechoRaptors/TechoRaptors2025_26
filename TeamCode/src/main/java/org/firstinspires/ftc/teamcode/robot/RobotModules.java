package org.firstinspires.ftc.teamcode.robot;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotModules {

    public HardwareMap localHardwareMap;
    public final double STOP_SPEED = 0.0;
    public Telemetry localTelemetry ;


    public RobotModules() {
    }


    public RobotModules(HardwareMap hwMap) {
        localHardwareMap = hwMap;
    }

    public RobotModules(HardwareMap hwMap, Telemetry tel) {
        localHardwareMap = hwMap;
        localTelemetry = tel;
    }

    public HardwareMap getLocalHardwareMap() {
        return localHardwareMap;
    }

    public void setLocalHardwareMap(HardwareMap localHardwareMap) {
        this.localHardwareMap = localHardwareMap;
    }

    public Telemetry getLocalTelemetry() {
        return localTelemetry;
    }

    public void setLocalTelemetry(Telemetry localTelemetry) {
        this.localTelemetry = localTelemetry;
    }
}
