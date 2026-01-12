// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class CommandLaunchpadController extends CommandJoystick {

    private static final int L2_RIGHT = 8;
    private static final int L2_LEFT = 7;
    private static final int L3_RIGHT = 6;
    private static final int L3_LEFT = 5;
    private static final int L4_RIGHT = 4;
    private static final int L4_LEFT = 3;

    public CommandLaunchpadController(int port) {
        super(port);
    }

    public Trigger l2Right() {
        return button(L2_RIGHT);
    }

    public Trigger l2Left() {
        return button(L2_LEFT);
    }

    public Trigger l3Right() {
        return button(L3_RIGHT);
    }

    public Trigger l3Left() {
        return button(L3_LEFT);
    }

    public Trigger l4Right() {
        return button(L4_RIGHT);
    }

    public Trigger L4Left() {
        return button(L4_LEFT);
    }

}
