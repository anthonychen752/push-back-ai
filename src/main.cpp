/*----------------------------------------------------------------------------*/
/*                                                                            */
/*   VEX AI Push Back - Main Program                                         */
/*   SEARCH → COLLECT → SCORE state machine                                  */
/*   Odometry + PID + Pure Pursuit                                           */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "robot-config.h"
#include "navigation.h"
#include <cmath>

namespace bex = vex;
using namespace bex;
brain::lcd &BrainScreen = Brain.Screen;

// ═══════════════════════════════════════════════════════════════════════════
// VEXCODE DEVICE CONFIG — mirrors robot-config.cpp for reference
// The actual device definitions are in robot-config.cpp (linked at build time)
// ═══════════════════════════════════════════════════════════════════════════

// Note: motors, sensors, Brain, Competition are declared in robot-config.h
// and defined in robot-config.cpp — do NOT redeclare them here

// ═══════════════════════════════════════════════════════════════════════════
// GLOBAL STATE
// ═══════════════════════════════════════════════════════════════════════════

StateMachine g_state;

// Control loop timing
const double DT = 0.02; // 20ms = 50 Hz

// ═══════════════════════════════════════════════════════════════════════════
// HELPER FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

// Calibrate sensors at match start
void calibrate()
{
    Inertial.calibrate();
    TrackingWheel.resetPosition(0);
    while (Inertial.isCalibrating())
    {
        wait(20, msec);
    }
    g_state.odom.reset(0, 0, Inertial.heading(rotationUnits::deg));
}

// Normalize angle to -180..180
double normalizeAngle(double a)
{
    while (a > 180)
        a -= 360;
    while (a < -180)
        a += 360;
    return a;
}

// Drive tank with left/right velocity (0..100 %)
void setDriveVel(double leftPct, double rightPct)
{
    LeftDrive.spin(fwd, leftPct, pct);
    RightDrive.spin(fwd, rightPct, pct);
}

void stopDrive()
{
    LeftDrive.stop();
    RightDrive.stop();
}

// Turn in place using PID
void turnToHeading(double targetDeg, double tolerance = 2.0, double timeout_ms = 5000)
{
    PID turnPID(TURN_KP, TURN_KI, TURN_KD);
    vex::timer turnTimer;
    turnTimer.resetTimer();
    double current;

    while (true)
    {
        current = Inertial.heading(rotationUnits::deg);
        double error = normalizeAngle(targetDeg - current);
        double output = turnPID.compute(error, DT);

        setDriveVel(-output, output);

        if (fabs(error) < tolerance)
            break;
        if (turnTimer.time(timeUnits::msec) > timeout_ms)
            break;
        wait(20, msec);
    }
    stopDrive();
}

// ═══════════════════════════════════════════════════════════════════════════
// AUTONOMOUS — Isolation Phase
// ═══════════════════════════════════════════════════════════════════════════

void auto_Isolation()
{
    calibrate();

    // State machine: SEARCH → COLLECT → SCORE
    g_state.state = RobotState::SEARCH;

    int collectCount = 0;
    const int COLLECT_TARGET = 3;

    while (true)
    {
        // Update odometry
        double trackingDeg = TrackingWheel.position(rotationUnits::deg);
        double headingDeg = Inertial.heading(rotationUnits::deg);
        g_state.odom.update(trackingDeg, headingDeg);

        switch (g_state.state)
        {

        case RobotState::SEARCH:
        {
            // Use pure pursuit to go to nearest collect zone
            Waypoint *zones = g_state.COLLECT_ZONES;
            int bestIdx = 0;
            double bestDist = 1e9;
            for (int i = 0; i < 3; i++)
            {
                double d = g_state.pp.distTo(zones[i].x, zones[i].y);
                if (d < bestDist)
                {
                    bestDist = d;
                    bestIdx = i;
                }
            }

            Waypoint target = zones[bestIdx];
            target.slow = true;
            g_state.pp.loadPath(&target, 1);

            bool arrived = g_state.pp.driveTo(target.x, target.y, 3.0);
            if (arrived)
            {
                g_state.state = RobotState::COLLECT;
            }

            // Apply wheel velocities from pure pursuit
            double lv, rv;
            g_state.pp.computeWheelVels(lv, rv);
            // Convert in/s to % (max ~24 in/s at 100%)
            setDriveVel(lv / BASE_SPEED * 100.0, rv / BASE_SPEED * 100.0);
            break;
        }

        case RobotState::COLLECT:
        {
            // Run intake, drive slowly forward
            Intake.spin(fwd, 70, pct);
            setDriveVel(10, 10);

            // Check for ball detection via Jetson AI vision
            AI_RECORD record;
            if (jetson_comms.get_data(&record) > 0 && record.detectionCount > 0)
            {
                // Got detection — count it
                (void)record; // suppress unused warning
                collectCount++;
            }

            // After collecting enough balls, go to goal
            if (collectCount >= COLLECT_TARGET)
            {
                Intake.stop();
                g_state.state = RobotState::DRIVE_TO_GOAL;
            }
            break;
        }

        case RobotState::DRIVE_TO_GOAL:
        {
            // Go to closest goal using pure pursuit
            Waypoint goals[2] = {g_state.SCORE_LEFT, g_state.SCORE_RIGHT};
            int bestIdx = 0;
            double bestDist = 1e9;
            for (int i = 0; i < 2; i++)
            {
                double d = g_state.pp.distTo(goals[i].x, goals[i].y);
                if (d < bestDist)
                {
                    bestDist = d;
                    bestIdx = i;
                }
            }

            Waypoint target = goals[bestIdx];
            target.slow = true;
            g_state.pp.loadPath(&target, 1);

            bool arrived = g_state.pp.driveTo(target.x, target.y, 3.0);
            if (arrived)
            {
                g_state.state = RobotState::SCORE;
            }

            double lv, rv;
            g_state.pp.computeWheelVels(lv, rv);
            setDriveVel(lv / BASE_SPEED * 100.0, rv / BASE_SPEED * 100.0);
            break;
        }

        case RobotState::SCORE:
        {
            // Back up and dump balls
            setDriveVel(-15, -15);
            wait(500, msec);
            stopDrive();

            // Score
            Intake.spin(fwd, 100, pct);
            wait(2, sec);
            Intake.stop();

            // Park
            g_state.state = RobotState::PARK;
            break;
        }

        case RobotState::PARK:
        {
            Waypoint park = g_state.PARK_POS;
            park.slow = true;
            g_state.pp.loadPath(&park, 1);
            bool arrived = g_state.pp.driveTo(park.x, park.y, 3.0);
            if (arrived)
                stopDrive();

            // Done — stay here
            wait(5, sec);
            return;
        }

        default:
            stopDrive();
            break;
        }

        // Debug — print to Brain screen
        BrainScreen.clearScreen();
        BrainScreen.print("State: %s", g_state.stateName());
        BrainScreen.printAt(0, 30, true, "X: %.1f  Y: %.1f  H: %.1f",
                          g_state.odom.x(), g_state.odom.y(), g_state.odom.theta());

        wait(20, msec);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// DRIVER CONTROL — placeholder (extend as needed)
// ═══════════════════════════════════════════════════════════════════════════

void usercontrol()
{
    controller Controller = controller();

    while (true)
    {
        // Tank drive
        double leftPct = Controller.Axis3.position(pct);
        double rightPct = Controller.Axis2.position(pct);
        setDriveVel(leftPct, rightPct);

        // Intake (button R1)
        if (Controller.ButtonR1.pressing())
        {
            Intake.spin(fwd, 70, pct);
        }
        else
        {
            Intake.stop();
        }

        // Hood (button R2)
        if (Controller.ButtonR2.pressing())
        {
            Hood.spin(fwd, 50, pct);
        }
        else
        {
            Hood.stop();
        }

        wait(20, msec);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// MAIN
// ═══════════════════════════════════════════════════════════════════════════

int main()
{
    // Request initial frame from Jetson AI vision
    jetson_comms.request_map();

    // Register competition callbacks
    Competition.autonomous(auto_Isolation);
    Competition.drivercontrol(usercontrol);

    // Default: run driver control until competition starts
    while (!Competition.isEnabled())
    {
        wait(20, msec);
    }
}
