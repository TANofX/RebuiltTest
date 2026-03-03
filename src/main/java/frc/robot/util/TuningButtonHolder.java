// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.input.controllers.XboxControllerWrapper;

/** Add your docs here. */
public class TuningButtonHolder {
    protected boolean active;
    private Trigger aTrigger;
    private Trigger bTrigger;
    private Command aCommand;
    private Command bCommand;

    public TuningButtonHolder(XboxControllerWrapper control, Command a, Command b) {
        active = false;

        aTrigger = control.A().and(this::isActive).toggleOnTrue(a);
        bTrigger = control.B().and(this::isActive).toggleOnTrue(b);
        
        aCommand = a;
        bCommand = b;
    }

    public boolean isActive() {
        return active;
    }

    public void setActive(boolean value) {
        active = value;

        if (!active) {
            if (aCommand.isScheduled()) aCommand.cancel();
            if (bCommand.isScheduled()) bCommand.cancel();
        }
    }
}
