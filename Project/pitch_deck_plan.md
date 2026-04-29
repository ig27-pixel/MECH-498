# RoboRoll Coatings Pitch Deck Plan

## Updated Assignment Shape

The deck should now be built in two parts:

- 4-5 main pitch slides that you would present live.
- A "Thank You" slide.
- An "Additional Materials" transition slide.
- 5-10 leave-behind slides that tell the fuller story without you presenting.

Target length: about 15 content slides, not counting title or transition slides. The deck is graded without the presentation, so every slide still needs to be understandable on its own.

## Core Story

RoboRoll Coatings builds compact robotic painters for interior construction and renovation crews. Painting is repetitive, labor intensive, quality-sensitive work that often happens when schedules are already tight. RoboRoll uses a portable 4-DOF manipulator, planned wall paths, and simulated controls to automate repeatable interior wall painting while keeping a human operator in charge of setup and supervision.

## Live Pitch Slides

### Title. RoboRoll Coatings

Purpose: Open with the company and product category.

Message: Robotic wall painting for faster, safer, more repeatable interior finishing.

Visual: `roboroll_demo_snapshot.png`

### 1. The Problem

Message: Interior painting is a manual bottleneck.

Use three punchy points:

- Repetitive wall coverage consumes crew hours.
- Quality depends on fatigue, reach, and setup consistency.
- Labor bottlenecks delay final construction turnover.

### 2. Why It Matters

Message: Construction needs focused automation that helps crews now.

Use three points:

- Painting is common across apartments, commercial spaces, and new builds.
- It is repetitive enough for automation but still needs human supervision.
- Better repeatability means fewer rework passes and more predictable schedules.

### 3. The RoboRoll Solution

Message: A portable 4-DOF painting robot follows planned wall paths.

Use three points:

- Base yaw aims the arm around the room.
- Shoulder, elbow, and wrist place the nozzle on target.
- IK turns wall points into smooth robot motion.

Visual: `dh_parameters.png` or a simplified robot render.

### 4. Product Demo

Message: The simulated robot already paints across two walls.

Use three points:

- Wall 1: five horizontal color stripes.
- Wall 2: face outline, eyes, and smile arc.
- Curved paths prove more than straight-line coverage.

Visual: `roboroll_demo_snapshot.png`

### 5. Why We Win / Ask

Message: RoboRoll starts with one focused construction task and grows from simulation to prototype.

Use three points:

- Narrow first market: repeatable interior wall finishing.
- Simulation-first design lowers hardware iteration cost.
- Seed goal: prototype paint delivery, wall sensing, and operator workflow.

## Transition Slides

### Thank You

Purpose: End the live pitch cleanly.

Suggested line: "RoboRoll Coatings: robotic painting for the next generation of construction crews."

### Additional Materials

Purpose: Tell the grader that the next slides are leave-behind material.

Suggested line: "The following slides provide the fuller technical, market, and roadmap story."

## Leave-Behind Slides

### A1. Beachhead Market

Tell the story: Start where the robot is easiest to justify.

Segments:

- Apartment turns and multifamily renovations.
- Commercial tenant improvements.
- New-build interior finishing crews.

### A2. Customer Workflow

Tell the story: RoboRoll supports the crew instead of replacing the whole job.

Steps:

- Operator rolls robot into room.
- Robot calibrates to wall plane.
- Software loads planned paint path.
- Human supervises coverage and refill/changeover.

### A3. Robot Architecture

Tell the story: The arm is intentionally simple enough for a first prototype.

Technical facts:

- 4 revolute joints.
- 500 mm base height.
- 700 mm upper link.
- 500 mm forearm link.
- 200 mm nozzle offset.

### A4. Kinematics Proof

Tell the story: The robot has the math needed to move from wall target to joint motion.

Technical proof:

- Forward kinematics computes end-effector pose.
- Inverse kinematics solves reachable wall targets.
- Joint limits prevent unrealistic poses.

### A5. Demo Path Details

Tell the story: The task is a miniature version of real coverage plus detail work.

Proof points:

- 343 rendered frames.
- Smoothstep interpolation between IK waypoints.
- Lift motions prevent unwanted diagonal paint marks.

### A6. Dynamics Proof

Tell the story: The design includes physical reasoning, not just animation.

Visuals:

- `dynamics_passive.png`
- `dynamics_controlled.png`

Proof points:

- Passive motion checks energy behavior.
- PD plus gravity feedforward controls motion.
- Torque and velocity plots support physically reasonable motion.

### A7. Development Roadmap

Tell the story: The next milestones are practical and hardware-focused.

Milestones:

- Paint delivery end effector.
- Wall sensing and calibration.
- Repeatability tests on drywall panels.
- Operator setup workflow.

### A8. Risks And Mitigations

Tell the story: The company understands what must be de-risked.

Examples:

- Overspray and surface quality: test nozzle spacing and flow rate.
- Wall localization: add sensing and calibration.
- Job-site setup time: design a simple operator workflow.

### A9. Funding Use

Tell the story: Funding turns the validated simulation into a prototype.

Use categories:

- Prototype arm and paint end effector.
- Sensors and calibration tooling.
- Test wall setup and materials.
- Software integration.

### A10. Closing Story

Tell the story: This is a focused construction automation wedge.

Closing message: RoboRoll begins with repeatable interior painting, proves value in controlled room-scale tasks, and expands toward broader finishing automation.

## Recording Strategy

If you record, keep it under 15 minutes at normal speed. A strong pacing target:

- Title: 30 seconds.
- Slides 1-5: about 6-8 minutes total.
- Leave-behind highlights: 4-5 minutes total.
- Closing: 30 seconds.

Do not spend much time explaining equations. Say what the technical work proves: the robot can reach wall targets, follow planned paths, and move under physically reasoned control.

