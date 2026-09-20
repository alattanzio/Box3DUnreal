# Box3DUnreal

<p align="center">
<img width="640" height="360" alt="box3d_test" src="https://github.com/user-attachments/assets/70152a87-7416-4efd-931f-063f6dff1ec6" />
</p>

## About

Hi, I'm Antonio Lattanzio. I'm a Principal Engineer at **Empty Vessel**, where we are currently working on **DEFECT**, an upcoming multiplayer immersive action game built with Unreal Engine.

🎮 **DEFECT**
[Steam Link](https://store.steampowered.com/app/2470010/DEFECT/)

I'm also the founder of **Mental Drink**, an independent game studio creating original games and experimental technology.

🎮 **Mental Drink Games**
[Steam Link](https://store.steampowered.com/search/?developer=Mental%20Drink)

---

# Overview

**Box3DUnreal** is an Unreal Engine plugin that integrates **Box3D** into Unreal Engine 5.

The goal of this project is to provide a clean and lightweight Unreal-friendly integration of Box3D, making it easier for developers to experiment with and build gameplay systems using a fast and robust physics engine.

> 🚧 **Early Development**
> This plugin is currently focused on the core Box3D integration. APIs, features, and documentation will continue to evolve.

---

# Features

* Box3D integrated into Unreal Engine 5
* Unreal Engine plugin architecture
* C++ and Blueprint API
* Physics events for designers: hit, contact, overlap, sleep/wake
* Box3D world and simulation support
* Lightweight integration layer
* Static mesh simulation support
* Raycasts, overlaps and shape casts against the Box3D world
* Joints: spherical, revolute, prismatic and weld, with limits, motors, springs and breaking
* Ragdolls built from a skeletal mesh's existing Physics Asset
* Kinematic character controller with spring-based ground contact
* Profiling through `stat box3d`, and Box3D's own debug renderer

More features and examples will be added as the project evolves.

---

# Why Box3D?

Box3D, created by **Erin Catto**, is a lightweight and robust physics engine used in many games and simulations.

While Unreal Engine provides Chaos Physics, Box3D can be useful for developers looking for:

* A lightweight physics solution
* Deterministic simulation
* Custom gameplay physics systems
* Experimentation with alternative physics engines

This project aims to make Box3D accessible from Unreal Engine while keeping the integration simple and flexible.

---

# Installation

Clone the repository including submodules:

```bash
git clone --recurse-submodules https://github.com/alattanzio/Box3DUnreal.git
```

Install the plugin either at the project level:

```text
<Project>/Plugins/Box3DUnreal
```

or at the Unreal Engine level:

```text
<UnrealEngine>/Engine/Plugins/Box3DUnreal
```

If installed at the engine level, enable the plugin from the Unreal Editor **Plugins** window if it is not enabled automatically.

![Enable Plugin](https://github.com/user-attachments/assets/3ca74b2c-f0f3-433d-9a43-ebb261496428)

Generate project files and build your Unreal project.

---

# Building the Box3D Library

Nothing extra to do — the first build compiles Box3D for you and links it. This section is
only for when you want to control *how* that happens.

## From source (default)

`Box3DUnreal.Build.cs` runs CMake on the vendored submodule once, caches the library under
`ThirdParty/Intermediate/<Platform>/`, and reuses it on every later build. Delete that folder
to force a rebuild.

It picks a `cmake` in this order:

1. `BOX3D_CMAKE` — full path to a specific executable.
2. **The CMake that ships with Unreal Engine**, `Engine/Extras/ThirdPartyNotUE/CMake` — the
   same one Epic's own third-party build scripts use. A source build of the engine has it;
   Epic Games Launcher installs ship only the licence notice, so the fallbacks below apply.
3. `cmake` on `PATH`.
4. Visual Studio's bundled CMake ("C++ CMake tools" component), then a standalone install.

## Linking a pre-built Box3D

To skip the CMake step entirely — no CMake on the machine, or a binary checked into your game
repo — point the plugin at an existing static library. Either set:

```text
BOX3D_PREBUILT_DIR=<path to the drop>
```

or place it where the plugin looks by default, which needs no environment variable:

```text
Box3DUnreal/ThirdParty/Prebuilt/<Win64|Mac|Linux>/
```

Expected layout (the library may also sit directly in the root):

```text
<root>/lib/box3d.lib       (Win64)   |   <root>/lib/libbox3d.a   (Mac/Linux)
<root>/include/box3d/*.h   optional — the submodule headers are used if absent
```

If the drop carries its own headers, the Box3D submodule does not have to be checked out at
all — handy for a shallow clone without `--recurse-submodules`.

Two things must match the plugin, or you get a link error:

* **Static, dynamic CRT.** On Windows build with `/MD`, like Unreal.
* **Double precision.** The plugin assumes double, which is what this repo's
  `ThirdParty/CMakeLists.txt` produces. For a single-precision library set
  `BOX3D_PREBUILT_PRECISION=float`, otherwise Box3D's deliberate ABI guard fails the link on
  `b3CreateWorldDoublePrecision`.

---

# Instructions

## Adding Box3D Simulation

To simulate a mesh using Box3D, add the **Box3DBody Component** to the Static Mesh Actor.

The component will register the mesh with the Box3D simulation and handle physics simulation independently from Chaos Physics.

![Box3DBody Component](https://github.com/user-attachments/assets/5310e80b-a24f-41e4-81c6-d864dd74fd3e)

Alternatively, you can add the **Box3DBody Component** directly from the Actor Components panel in the Outliner.

![Adding Box3DBody Component](https://github.com/user-attachments/assets/8df90aab-c257-4f5e-bfc9-55cdea646532)

---

## Collision Setup

If **Convex** is selected, Box3D will import the Static Mesh simple collision geometry and use it for the Box3D simulation.

![Convex Collision Setup](https://github.com/user-attachments/assets/7a059937-80e6-4d0f-b0cb-04c31312fd2b)

---

## Chaos Physics Compatibility

Chaos Physics remains enabled in Unreal Engine, but the object will **not** be simulated by Chaos.

From Chaos' perspective, the mesh remains static.

If you want Chaos to completely ignore the object, set the Collision Profile to:

**No Collision**

![No Collision Setting](https://github.com/user-attachments/assets/b40c395d-887e-4d4d-8fa0-8e190cb88c42)

![Uploading box3d_test.gif…]()
![Collision Example](https://github.com/user-attachments/assets/760fb108-8c4b-4125-8a48-61d3dc4268a7)

---

## Physics Events

Events work the same way they do in Chaos: tick a checkbox on the **Box3DBody Component**,
then bind the matching event in the actor's Blueprint graph.

| Checkbox | Events |
| --- | --- |
| **Simulation Generates Hit Events** | `On Box3D Hit` |
| **Generate Contact Events** | `On Box3D Begin Contact` / `On Box3D End Contact` |
| **Generate Overlap Events** | `On Box3D Begin Overlap` / `On Box3D End Overlap` |
| **Is Trigger** | turns the body into a trigger volume (never blocks) |
| **Generate Sleep Events** | `On Box3D Sleep` / `On Box3D Wake` |

**Hit** is the one most gameplay wants. It only fires for collisions above a speed threshold
and carries the impact location, normal and closing speed, so you can scale damage, sound or
decals by how hard the impact was. **Contact** fires for every touch however gentle — use it
only when the touch itself is the point.

Two things behave differently from Chaos, both inherited from Box3D:

* **Hit and contact only need the checkbox on one of the two bodies.** A prop can hear about
  hitting level geometry that has no events of its own.
* **Overlap needs it on both.** A trigger is blind to anything without *Generate Overlap
  Events*, so tick it on the trigger *and* on whatever should be detected. A trigger should be
  Static or Kinematic — a Dynamic one never collides, so it just falls out of the level.

Events fire on the server only, since only the server simulates. Replicate the reaction, not
the event.

For a single handler that hears every impact in the level (impact audio, decals), bind
`On Any Box3D Hit` on the **Box3D Subsystem** instead.

---

## Blueprint API

The body component exposes the usual physics verbs: `Add Impulse`, `Add Impulse At Location`,
`Add Force`, `Add Torque`, `Add Angular Impulse`, `Set`/`Get Linear Velocity`,
`Set`/`Get Angular Velocity`, `Set Gravity Scale`, `Set Sleep Enabled`, `Teleport Body`,
`Get Body Mass`, `Is Body Awake`, `Wake Body`.

The subsystem exposes the world: raycasts, overlaps and shape casts, `Get`/`Set Gravity`,
`Apply Radial Impulse`, plus `Is Simulation Authority` — check that before trusting any
Box3D state, because a client has no bodies and everything comes back empty.

---

## Joints

Four joint types are exposed: **spherical** (ball socket, with cone and twist limits),
**revolute** (hinge), **prismatic** (slider) and **weld**. All support limits, motors,
springs and optional breaking.

You never build joint frames by hand. Each create call takes a world-space anchor and an
axis, and derives both body-local frames from them — so the joint's rest state is whatever
relative pose the two bodies are already in, the same "constrain them where they stand"
behaviour as Chaos.

From C++ or Blueprint, via `Box3DJointLibrary`:

```cpp
FBox3DJointSettings Settings;
FBox3DRevoluteJointSettings Hinge;
Hinge.bEnableLimit = true;
Hinge.LowerAngle = -90.0f;   // degrees
Hinge.UpperAngle = 0.0f;

const FBox3DJointHandle Door = UBox3DJointLibrary::CreateRevoluteJoint(
    FrameBody, DoorBody, HingeWorldLocation, FVector::UpVector, Settings, Hinge);
```

Or add a **Box3D Joint Component** and fill in its `Joints` array in the details panel.
Joints are created one tick after BeginPlay, so both bodies exist whatever order their
components initialised in.

Set `Break Force` (N) or `Break Torque` (N·m) and tick `Breakable` to have a joint give
way; `On Box3D Joint Break` on the subsystem fires once the step loop is over.

Units are Unreal's throughout: cm, degrees, deg/s.

---

## Ragdolls

Add a **Box3D Ragdoll Component** to an actor with a skeletal mesh and call `Start Ragdoll`.
It reads the mesh's existing `UPhysicsAsset` and builds one dynamic body per physics body
and one spherical joint per constraint, using the asset's own swing and twist limits.

Nothing is authored twice — a rig already set up for Chaos ragdolls works unmodified.

```cpp
Ragdoll->StartRagdoll();
Ragdoll->AddImpulseAtLocation(ShotDirection * 40000.0, HitLocation);
```

The result is written back through the mesh's component-space bone transforms, so it
renders through the ordinary skinning path. Bones with no physics body keep their last
animated pose and follow their parent.

Two import caveats worth knowing: a Physics Asset's swing cone is *elliptical* while
Box3D's is circular, so the larger axis is taken (trim it with `Swing Limit Scale`); and
rotated **box** elements import unrotated and log a warning — capsules and spheres, which
are nearly all ragdoll geometry, rotate correctly.

---

## Character Controller

**Box3D Character Component** is a kinematic capsule mover built on Box3D's character API —
plane collection, plane solving, then a swept move. Ground contact is a damped spring
rather than a snap, which is what handles stairs and slopes without a separate step-up
pass; `Pogo Rest Length Scale` effectively sets the step height.

Movement is Quake-style: friction, then acceleration toward a desired velocity capped at
`Max Speed`. It runs on the simulation's fixed timestep, so unlike
`CharacterMovementComponent` it is deterministic and fits the rollback engine.

Feed it input each frame and it does the rest:

```cpp
Character->SetMoveInput(Forward * ForwardAxis + Right * RightAxis);
Character->Jump();
```

`ABox3DCharacterPawn` is a ready-made third-person pawn wrapping it, which is what
`box3d.CharacterDemo` spawns.

---

# Console Commands

## Spawn Box3D Test Object

```console
box3d.Spawn
```

## Demos

Each spawns in front of the player and needs no level content.

```console
box3d.JointDemo chain 12    // rope of capsules on spherical joints
box3d.JointDemo bridge 10   // hinged plank walkway, anchored both ends
box3d.JointDemo newton 5    // Newton's cradle
box3d.JointDemo motor       // motorised hinge arm
box3d.CharacterDemo         // spawn and possess a kinematic character
```

## Debug Rendering

Enable:

```console
box3d.DebugDraw 1
```

Disable:

```console
box3d.DebugDraw 0
```

Box3D also ships its own renderer, which draws what the solver actually sees — contact
points and normals, joint frames, islands, sleep state, and real static shapes rather than
their bounds. It takes a bitmask:

```console
box3d.NativeDraw 67
```

`1` shapes, `2` joints, `4` joint extras, `8` bounds, `16` mass, `32` sleep, `64` contacts,
`128` contact normals, `256` contact forces, `512` islands, `1024` graph colours. `67` is
shapes + joints + contacts. Authority only, since a client has no world to walk.

## Profiling

```console
stat box3d
```

Per-phase solver timings come from Box3D itself — step, broadphase, collide, solve,
continuous, sensors — alongside body, shape, contact, joint and island counts, and how many
fixed steps the frame took. These are Box3D's own numbers, not a wrapper's estimate, so
they are directly comparable against `stat physics`.

## Self-Checking Tests

These need no level content and log a pass/fail tally.

```console
box3d.QueryTest
box3d.EventTest
```

---

# Compatibility

Tested with:

* Unreal Engine **5.7.x**
* Unreal Engine **5.8.x**

---

# Documentation

Deeper docs live in `Box3DUnreal/Docs/`:

* **[Architecture](Box3DUnreal/Docs/Architecture.md)** — the layers, what owns which Box3D
  handle, and the coordinate and unit conventions. Worth reading before contributing.
* **[Joints and Ragdolls](Box3DUnreal/Docs/Joints.md)** — joint types and settings, breaking,
  and the Physics Asset import with its caveats.
* **[Integration Build Plan](Box3DUnreal/Docs/Box3DIntegration.md)** — the full design doc:
  authority contract, precision decisions, the determinism analysis and its measured
  results, networking, and build setup.

## A note on conventions

Two things that trip people up when reading the code:

**Handedness.** Unreal is left-handed, Box3D right-handed. The conversion negates Y, which
keeps gravity on Z. Because that map is a reflection, pseudovectors — angular velocity,
torque, angular impulse — negate X and Z instead of Y. Copying components straight across
without this looks fine on symmetric scenes and goes wrong the moment chirality matters,
such as which way a motor or wheel turns.

**Units.** Selectable with `box3d.LengthUnits`: `1` for meters (default — positions are
scaled at the boundary, and the solver keeps its float precision) or `100` for centimeters
(`b3SetLengthUnitsPerMeter`, no scaling, at the cost of global process state). Either way
every conversion goes through a single header.

---

# Current Limitations

This project is currently focused on the core Box3D integration.

Current limitations:

* No Chaos ↔ Box3D physics interaction
* Server-authoritative only — clients display replicated movement, no client prediction yet
* No vehicles
* The simulation steps on the game thread
* Limited editor tooling

Additional features will be added as the integration evolves.

---

# Roadmap

Rough order of value, not a schedule.

* **Vehicles.** Box3D has a wheel joint; a drivable vehicle on top of it would be the next
  big gameplay system, and the last common one that still has to fall back to Chaos.
* **Async / off-thread stepping.** The step is a clean unit of work and could run alongside the
  frame with the transform write-back on the game thread. Note the parallelism has to come from
  stepping beside the frame, not from more solver workers — those repartition the constraint
  graph and break determinism.
* **Continuous collision and per-axis motion locks.** One property each on the component.
  CCD matters as soon as anything fast-moving becomes a Box3D body.
* **Landscape height fields.** Tri-mesh collision already works; a real height field would be
  faster.
* **Client-side prediction and rollback.** The snapshot, hash and replay pieces exist and are
  proven headless; wiring them to a live client is the remaining work.
* **Pre-solve contact events** for one-way platforms and conveyors. These run on solver
  threads, so they can only ever be a C++ callback, never a Blueprint event.
* **Example project.**

---

# Contributing

Contributions are welcome!

If you find an issue, have an idea, or want to improve the plugin:

* ⭐ Star the repository
* 🐛 Report bugs
* 💡 Suggest improvements
* 🔀 Submit pull requests

Whether it is improving documentation, adding features, or fixing issues, every contribution is appreciated.

---

# Connect & Support

If you enjoy this project, feel free to follow my work on GitHub or connect on LinkedIn.

I'm also the founder of **Mental Drink**, an independent game studio creating original games and experimental technology.

If you'd like to support independent development, consider wishlisting our games on Steam. Every wishlist helps developers continue creating games and open-source technology.

LinkedIn:
https://www.linkedin.com/in/antoniolattanzio/

🎮 **DEFECT**
[Steam Link](https://store.steampowered.com/app/2470010/DEFECT/)

🎮 **Mental Drink Games**
[Steam Link](https://store.steampowered.com/search/?developer=Mental%20Drink)

---

# Third-Party

This plugin integrates **Box3D**, created by Erin Catto.

Please refer to the official Box3D project for licensing information.

---

# License

The Unreal Engine integration is released under the MIT License.

Box3D remains under its own license.
