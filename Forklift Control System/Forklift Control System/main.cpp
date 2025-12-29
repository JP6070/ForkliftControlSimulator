#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string>
#include <thread>

// PLC-style "scan" data

struct Inputs {
    bool cmdUp = false;
    bool cmdDown = false;
    bool cmdHold = false;      // optional explicit hold command
    bool estop = false;
    bool resetFault = false;

    bool topLimit = false;
    bool bottomLimit = true;   // start at bottom in this sim

    double loadKg = 0.0;       // for overload detection
};

struct Outputs {
    bool motorEnable = false;
    int motorDir = 0;          // +1 up, -1 down, 0 none
    bool brakeEngaged = true;  // true = brake on
    bool faultLamp = false;
};


// Faults with explicit priority

enum class FaultCode : int {
    None = 0,
    LimitViolation = 10,
    Overload = 20,
    EmergencyStop = 30,
};

static const char* faultToString(FaultCode f) {
    switch (f) {
    case FaultCode::None: return "None";
    case FaultCode::LimitViolation: return "LimitViolation";
    case FaultCode::Overload: return "Overload";
    case FaultCode::EmergencyStop: return "EmergencyStop";
    }
    return "Unknown";
}

// Higher number = higher priority
static int faultPriority(FaultCode f) {
    return static_cast<int>(f);
}

struct FaultManager {
    FaultCode latched = FaultCode::None;

    void clear() { latched = FaultCode::None; }

    void latch(FaultCode f) {
        if (faultPriority(f) > faultPriority(latched)) {
            latched = f;
        }
    }

    bool hasFault() const { return latched != FaultCode::None; }
};

// Lift model + PLC state machine

enum class LiftState {
    Holding,
    Lifting,
    Lowering,
    Faulted,
};

static const char* stateToString(LiftState s) {
    switch (s) {
    case LiftState::Holding: return "Holding";
    case LiftState::Lifting: return "Lifting";
    case LiftState::Lowering: return "Lowering";
    case LiftState::Faulted: return "Faulted";
    }
    return "Unknown";
}

struct LiftPlant {
    // Simple physical-ish model (units arbitrary but consistent)
    double position = 0.0;     // 0 = bottom, 1 = top
    double velocity = 0.0;     // units per second

    // "Actuators"
    double targetVel = 0.0;    // commanded velocity

    // Update plant each tick
    void step(double dt) {
        // Smooth towards target velocity (a tiny bit of inertia)
        const double accel = 3.0; // units/s^2
        double dv = targetVel - velocity;
        double maxDv = accel * dt;
        dv = std::clamp(dv, -maxDv, maxDv);
        velocity += dv;

        position += velocity * dt;
        position = std::clamp(position, 0.0, 1.0);

        // If we hit the ends, clamp velocity
        if (position <= 0.0 && velocity < 0.0) velocity = 0.0;
        if (position >= 1.0 && velocity > 0.0) velocity = 0.0;
    }
};

struct LiftController {
    // Tunables
    const double maxLoadKg = 1200.0;
    const double liftSpeed = 0.35;
    const double lowerSpeed = 0.30;
    const double safeStopSpeedEps = 0.01;

    LiftState state = LiftState::Holding;
    FaultManager faults;

    // Debounce-like memory
    bool lastTopLimit = false;
    bool lastBottomLimit = true;

    Outputs update(double dt, const Inputs& in, LiftPlant& plant) {
        Outputs out{};
        out.brakeEngaged = true;

        // ---- 1. Latch faults (priority-based) ----
        if (in.estop) {
            faults.latch(FaultCode::EmergencyStop);
        }
        if (in.loadKg > maxLoadKg) {
            faults.latch(FaultCode::Overload);
        }

        // Limit/sensor consistency + "commanding into a limit"
        if (in.topLimit && in.bottomLimit) {
            faults.latch(FaultCode::LimitViolation);
        }
        else {
            if (state == LiftState::Lifting && in.topLimit)    faults.latch(FaultCode::LimitViolation);
            if (state == LiftState::Lowering && in.bottomLimit) faults.latch(FaultCode::LimitViolation);

            if (in.cmdUp && in.topLimit)    faults.latch(FaultCode::LimitViolation);
            if (in.cmdDown && in.bottomLimit) faults.latch(FaultCode::LimitViolation);
        }

        // ---- 2. Allow reset ----
        // Only allow reset when E-stop is released and the lift is stationary-ish.
        if (in.resetFault && !in.estop && std::abs(plant.velocity) < safeStopSpeedEps) {
            faults.clear();
        }

        // ---- 3. State transitions ----
        if (faults.hasFault()) {
            state = LiftState::Faulted;
        }
        else {
            const bool up = in.cmdUp;
            const bool down = in.cmdDown;

            if (up && !down && !in.topLimit) {
                state = LiftState::Lifting;
            }
            else if (down && !up && !in.bottomLimit) {
                state = LiftState::Lowering;
            }
            else {
                state = LiftState::Holding;
            }
        }

        // ---- 4. Outputs + safe stopping ----
        switch (state) {
        case LiftState::Faulted:
            plant.targetVel = 0.0;
            out.motorEnable = false;
            out.motorDir = 0;
            out.brakeEngaged = true;
            out.faultLamp = true;
            break;

        case LiftState::Holding:
            plant.targetVel = 0.0;
            out.motorEnable = false;
            out.motorDir = 0;
            out.brakeEngaged = true;
            out.faultLamp = false;
            break;

        case LiftState::Lifting:
            if (in.topLimit) {
                plant.targetVel = 0.0;
                out.motorEnable = false;
                out.motorDir = 0;
                out.brakeEngaged = true;
            }
            else {
                plant.targetVel = +liftSpeed;
                out.motorEnable = true;
                out.motorDir = +1;
                out.brakeEngaged = false;
            }
            out.faultLamp = false;
            break;

        case LiftState::Lowering:
            if (in.bottomLimit) {
                plant.targetVel = 0.0;
                out.motorEnable = false;
                out.motorDir = 0;
                out.brakeEngaged = true;
            }
            else {
                plant.targetVel = -lowerSpeed;
                out.motorEnable = true;
                out.motorDir = -1;
                out.brakeEngaged = false;
            }
            out.faultLamp = false;
            break;
        }

        (void)dt;
        lastTopLimit = in.topLimit;
        lastBottomLimit = in.bottomLimit;

        return out;

        }
    
};

    // Console Simulation
static void printHelp() {
    std::cout <<
        "Commands:\n"
        "  u  = command up\n"
        "  d  = command down\n"
        "  h  = hold\n"
        "  s  = stop commands (clear u/d/h)\n"
        "  e  = toggle emergency stop\n"
        "  r  = reset fault (only if stopped + estop released)\n"
        "  l <kg> = set load kg (e.g. l 900)\n"
        "  q  = quit\n";
}

int main() {
    LiftPlant plant{};
    LiftController ctrl{};
    Inputs in{};
    Outputs out{};

    const double dt = 0.02; // 20ms fixed update loop

    printHelp();

    bool quit = false;
    while (!quit) {
        // ---- Reset is a pulse: default false each cycle ----
        in.resetFault = false;

        // ---- Non-blocking-ish input (do this BEFORE controller scan) ----
        static int inputPoll = 0;
        if ((inputPoll++ % 25) == 0) { // every 0.5s
            std::cout << "> " << std::flush;
            std::string line;
            if (std::getline(std::cin, line)) {
                if (line == "q") quit = true;
                else if (line == "u") { in.cmdUp = true;  in.cmdDown = false; in.cmdHold = false; }
                else if (line == "d") { in.cmdDown = true; in.cmdUp = false;  in.cmdHold = false; }
                else if (line == "h") { in.cmdHold = true; in.cmdUp = false;  in.cmdDown = false; }
                else if (line == "s") { in.cmdUp = in.cmdDown = in.cmdHold = false; }
                else if (line == "e") { in.estop = !in.estop; }
                else if (line == "r") { in.resetFault = true; } // now ctrl.update will see it this cycle
                else if (line.size() >= 2 && line[0] == 'l') {
                    try { in.loadKg = std::stod(line.substr(1)); }
                    catch (...) { std::cout << "Bad load value.\n"; }
                }
                else if (line == "help") printHelp();
                else std::cout << "Unknown command. Type 'help'.\n";
            }
        }

        // ---- Update derived inputs (limit switches) from plant position ----
        in.bottomLimit = (plant.position <= 0.0001);
        in.topLimit = (plant.position >= 0.9999);

        // ---- Controller scan (NOW it can see resetFault) ----
        out = ctrl.update(dt, in, plant);

        // ---- Plant update ----
        if (out.brakeEngaged) plant.targetVel = 0.0;
        plant.step(dt);

        // ---- Status print (every 200ms) ----
        static int tick = 0;
        if ((tick++ % 10) == 0) {
            std::cout << std::fixed << std::setprecision(3)
                << "pos=" << plant.position
                << " vel=" << plant.velocity
                << " state=" << stateToString(ctrl.state)
                << " fault=" << faultToString(ctrl.faults.latched)
                << " top=" << in.topLimit
                << " bot=" << in.bottomLimit
                << " load=" << in.loadKg
                << " estop=" << in.estop
                << "\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}
