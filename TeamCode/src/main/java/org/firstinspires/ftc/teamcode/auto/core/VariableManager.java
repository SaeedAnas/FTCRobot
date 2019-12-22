package org.firstinspires.ftc.teamcode.auto.core;


import org.firstinspires.ftc.robotcore.external.Telemetry;

public class VariableManager {
    public static VariableManager vars;
    private Telemetry telemetry;
    private VariableManager(Telemetry tele) {
      this.telemetry = tele;
    }

    public synchronized static VariableManager getInstance(Telemetry tele) {
        if (vars == null) {
            vars = new VariableManager(tele);
        }
        return vars;
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }
}
