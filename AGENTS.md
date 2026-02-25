# AGENTS.md: Directives for AI Contributors

This repository enforces strict rules for AI agents contributing to hardware-linked software. The primary failure mode of previous iterations was the divergence between the AI's internal model of the system and the physical reality of the hardware.

You must adhere to the following directives when writing, refactoring, or reviewing code in this project.

## 1. Hardware Reality over Assumptions

Agents frequently make incorrect assumptions about physical connections, pinouts, and hardware behavior.

* **Never assume how hardware is plugged in correctly.** Pin numbers, I2C addresses, PWM channels, and motor polarities must be strictly parameterized and exposed in configuration files. Do not hardcode these values in the logic layers.
* **Assume hardware components lie.** Sensors drift, I2C buses hang, and motors stall. Implement timeouts, watchdogs, reasonable retries, and explicit error handling for all hardware interfaces.
* **Battery Degradation:** The battery levels will change over time, so that will be hard to account for. Motor PWM outputs will yield less physical torque as voltage drops.
* Swallowing an error is a crime.
* Relaxing (like allowing mocks to the hardware) require explicit flags, the base case should require all hardware to work and fail loudly if not.

## 2. Coordinate Systems and Directionality

Debates over "which way is forward" consume unnecessary cycles and lead to sign-flip errors scattered across the codebase.

* **Establish the Frame of Reference Once:** Define a single, explicit coordinate system (e.g., X is forward, Z is up) at the boundary of the application.
* Assume any Gyros and Accelerometers aren't aligned with the frame of reference: They are likely mounted at an approximately right angle, but might be turned 90/180/270, or upside down and backwards.
* **Isolate Inversions:** All sensor inversions (e.g., IMU mounting orientation) and actuator inversions (e.g., motor wiring polarity) must be handled in exactly one place: the hardware abstraction layer. The core control logic must only deal in normalized, mathematically pure values. Do not let `-1` multipliers leak into the core control loops.

## 3. Architectural Rigor (DAGs and Pipelines)

Complex architectures are fragile. If introducing advanced execution models like Directed Acyclic Graphs (DAGs) for discovery pipelines or state machines:

* **Verify Dependencies Explicitly:** You must guarantee execution order. A pipeline will fail silently if a node consumes data before its prerequisite node has updated it.  It must instead fail loudly.
* **Fail Loudly on Cycles:** Implement strict checks for circular dependencies and missing prerequisites.
* **Keep Control Loops Flat:** Unless a DAG provides measurable utility, prefer flat, deterministic control loops. Complexity must justify its existence with data.

## 4. Telemetry and Ground Truth

LLMs cannot see the physical robot. When debugging, an LLM will invent a narrative about what is happening unless forced to look at data. This **must** be aggressively prevented.

* **Log Actual Values:** Logging is not optional. You must log raw sensor inputs (without overloading the context window), filtered sensor values, target setpoints, intermediate controller terms (e.g., P, I, and D components separately), and the final output commands.
* **Do Not Guess Physics:** If a physical behavior is incorrect (e.g., the robot falls over or oscillates), do not rewrite the control logic based on a theoretical assumption. Request the log data first.

## 5. Defense in Depth

Hardware requires failsafes. Code that works perfectly in a test suite will fail when a battery drops voltage or a wire comes loose.

* **Fail Loud and Fast:** If a critical sensor stops responding or a constraint is violated (e.g., max tilt angle exceeded), the system must immediately disarm actuators and halt.
* **State Machine Boundaries:** If using state machines for behavior, ensure transition conditions are mutually exclusive and logically complete. Undefined states in hardware lead to physical damage.

## 6. You don't have all the answers, but you know some things to be true

All numbers in this section (10 degress, 20 degrees, 10% power, etc) are assumptions that need to be verified.

1. The robot has two wheels of approximately the same radius and approximately the same motor power.  They are side-by-side.
2. There isn't a defined "front" and "back" - pick one and assume the motors are randomly attached, and may not be facing the same way.
3. The gyro is mounted in a near-right-angle, so if you are seeing a tilt more than 10 degrees, that means the robot is actually tilted.
4. There are unusual features: This robot has "lean forward bumpers/training wheels" and "lean backwards bumpers/training wheels" that mostly keep it from falling completely over.  These are around 20 degrees, but aren't symmetrical. Note that **a 20-degree lean is a valid state, not a crash**.
5. The robot can drive around on a bumper skidding on the ground, but this is like a toddler crawling.  Balancing on two wheels without using the bumpers after standing up is like the toddler walking (and toddlers love to walk!)
6. The robot is strong enough that a hard acceleration from standstill (or maybe it requires a quick inversion of direction?) to "flop" from front to back bumper and vice-versa.  The strength needed is somewhere between 10% and 100%, but we aren't sure where.
7. Before the PID is active, it will always be flopped forward or backwards, happily skidding around on the two wheels plus a front or back training wheel.
8. Both gyro readings AND accelerometer readings are necessary to differentiate "spinning" from "tilting".
9. **Crash State:** A tilt > 50 degrees is the true crash state requiring intervention. This only counts once the gyro's orientation is confidently determined.

## 7. The "Tabula Rasa" Experimental Protocol

The agent "doesn't have all the answers" (Section 6), so it must use the scientific method to find them. **NEVER ASSUME OR GUESS THE HARDWARE.**

* **Sequential Discovery:** Follow a strict 7-phase state machine that forces the LLM to run sequential, pessimistic physics experiments:
    1. Spark of Life
    2. Sense of Down
    3. Stiction
    4. Polarity
    5. Left/Right
    6. Trim
    7. Balance Point
* **Atomic Experiments:** If uncertain, write an atomic experiment, observe the result, deduce the fact, and save it.

## 8. Strict Data Structure Constraints

* **Use `balance_bot.utils.Vector3`:** Always use `balance_bot.utils.Vector3` for 3D spatial data. It is an immutable NamedTuple. Do not use dictionaries or raw tuples. Enforcing a specific data structure prevents type errors and messy refactoring loops.
* **Libraries:** `simple-pid` and `PyGLM` should be used.

## 9. The "Do No Harm" Refactoring Rule

* **Stateless Vacuum:** Process code in a "stateless vacuum," only applying changes that move the code to the target state.
* **No Stylistic Changes:** Explicitly forbid stylistic changes for the sake of novelty. AI agents will often unnecessarily rewrite functional boilerplate or change variable names while trying to fix a bug, causing regressions in unrelated systems.
* **Separation of Concerns:** Never mix doc updates with code updates. Lots of agents are working in parallel, and every "extra cleanup" causes a merge conflict.

## 10. Tooling and Environment Directives

* **Linting:** Run lint checks before commits using `uv run ruff check`.
* **Testing:** Run tests using `uv run pytest`.
* **Mocks:** If you are using `uv` or specific mock flags, the agent needs to know the exact CLI commands to test its own code. Use the `--allow-mocks` flag when necessary: `uv run pytest --allow-mocks`.
