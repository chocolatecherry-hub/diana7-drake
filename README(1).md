# diana7-drake

Simulation of an **Agile Robots Diana 7** (7-DoF collaborative arm) in [Drake](https://drake.mit.edu/), with a welding table and a test product, as the first step towards **unscrewing electronic products** with the real robot.

Master thesis project, KU Leuven. Authors: Emre (`chocolatecherry-hub`) and `<partner name>`.

---

## 1. What this project does (the big picture)

The robot is modelled as a **system of blocks in a Drake diagram**. We build it up in small steps, so each script adds one idea:

| Step | Script | What it adds | Status |
|---|---|---|---|
| 1 | `scripts/01_view_robot.py` | Load the Diana 7 into a `MultibodyPlant`, view it in Meshcat, zero-torque (falls under gravity) | works (tested by Emre) |
| 2 | `scripts/02_hold_pose.py` | Add an `InverseDynamicsController` so the arm holds a pose | written, **not yet verified** |
| 3 | `scripts/03_table_and_robot.py` | Welding table + robot bolted to it, coordinate frames drawn in Meshcat | written, **not yet verified** |
| 4 | `scripts/04_unscrew_sequence.py` | Plate with screw holes; inverse kinematics; robot visits every hole on a fixed time schedule | written, **not yet verified** |
| 5 | `scripts/05_state_machine.py` | Same task, but decided at run time by our own `LeafSystem` state machine (moves on only when it has arrived, goes to `FAULT` if it does not) | written, **not yet verified** |

(Update the Status column when you run a script. Please do not trust a script that is not marked as working.)

Planned next: collision-aware IK and straight tool paths, real hole positions from an Intel RealSense L515 point cloud, and a hardware interface to the real robot (see section 7).

---

## 2. Folder structure

```
diana7-drake/
├── README.md                  this file
├── requirements.txt           Python packages (pinned versions)
├── .gitignore
├── models/
│   ├── diana7/
│   │   ├── diana7.urdf        robot description (GENERATED, see section 5; do not edit by hand)
│   │   └── meshes/...         .obj meshes (converted from the vendor's .stl)
│   ├── table/
│   │   └── welding_table.urdf black box, hand-written; measure the real table and edit
│   └── object/
│       └── plate_with_holes.urdf   grey test plate, hand-written
├── src/
│   └── scene.py               helpers: add_table(), add_diana7(), add_object() (add models + weld frames)
└── scripts/
    ├── 01_view_robot.py ... 05_state_machine.py    the steps from section 1
    └── convert_stl_to_obj.py  one-time tool: vendor .stl meshes -> .obj (Drake cannot load .stl)
```

---

## 3. Setup

Needs Linux (we use Ubuntu) and Python 3.

```bash
git clone https://github.com/chocolatecherry-hub/diana7-drake.git
cd diana7-drake

python3 -m venv ~/drake-env                 # a private Python environment, kept OUTSIDE the repo
~/drake-env/bin/pip install -r requirements.txt
sudo apt install graphviz                   # only needed to draw Drake's block diagram as SVG
```

Run any script from the repository folder, for example:

```bash
~/drake-env/bin/python scripts/01_view_robot.py models/diana7/diana7.urdf
~/drake-env/bin/python scripts/03_table_and_robot.py
~/drake-env/bin/python scripts/05_state_machine.py 2>&1 | grep -v WARNING
```

Each script prints a Meshcat address (usually `http://localhost:7000`). Open it in a browser to see the scene. `grep -v WARNING` hides harmless URDF warnings (Drake ignores the vendor's `safety_controller` tags and joint friction).

---

## 4. Concepts you need to read the code

**Drake diagram.** Everything is a *system* (a block with input and output ports); a *diagram* is blocks wired together. The simulated world is: `MultibodyPlant` (physics) + `SceneGraph` (geometry) + `MeshcatVisualizer` (browser view) + a controller. Blocks hold no numbers; the *context* holds the current state of the whole diagram.

**Frames.** A frame is a coordinate system. `X_AB` means "pose of frame B expressed in frame A". We chain them:

```
world W  --X_WT-->  table_top T  --X_TB-->  robot base B  -->  link1 ... link8 (flange)
                          \--X_TO-->  plate frame O  -->  hole positions
```

- World `W`: origin on the floor, z up.
- **Table frame `T`**: origin at the **centre of the table's top surface**; x along the length, y along the width. The table box hangs *below* this frame.
- **Plate frame `O`**: origin at the centre of the plate's **top face**, so the plate is lifted by its thickness (`OBJECT_THICKNESS`) to sit on the table.
- `WeldFrames(parent, child, X_PC)` glues two frames together with a fixed pose. Using the same "frame on the top face" convention for the table and the plate keeps all measurements simple.

**State machine (step 5).** `UnscrewStateMachine` is our own `LeafSystem`. Its memory is the current step; every 5 ms it checks the measured joint angles and decides whether to move to the next step (`APPROACH_HOLE_n -> DESCEND_HOLE_n -> UNSCREW_HOLE_n -> ASCEND_HOLE_n -> ... -> RETURN_HOME -> DONE`) or to go to `FAULT`. Step 4 only replays a time schedule and never looks at the robot.

---

## 5. Where the Diana 7 model comes from

Vendor repo: <https://github.com/AgileRobots-APAC/diana_description> (a ROS 2 package; meshes are `.stl`). We **did not** edit it. `models/diana7/` was produced once like this, so you do **not** need to repeat it:

1. `xacro diana7.urdf.xacro > diana7.urdf` (the vendor description is an xacro template; because we do not use ROS, `$(find diana_description)` was resolved with a tiny stand-in for the ROS package index).
2. `scripts/convert_stl_to_obj.py` converted every `.stl` mesh to `.obj` and rewrote the mesh paths in the URDF to relative paths.

The URDF has real masses and inertias, and joint limits. It has no actuators, so `src/scene.py` adds one per revolute joint.

> **Licence:** check the vendor repo's `LICENSE` before making this repository public, since it contains converted copies of their meshes.

---

## 6. Things that are placeholders and must be measured

- `models/table/welding_table.urdf`: table size (1.2 x 0.8 x 0.9 m is a guess). If you change the height, also change `TABLE_TOP_HEIGHT` in the scripts.
- `ROBOT_XY`, `ROBOT_YAW_DEG`: where the robot base sits on the real table, measured from the centre of the top surface.
- `OBJECT_XY`, `OBJECT_THICKNESS`, `HOLES_IN_OBJECT`: the test plate (not a real product yet).
- `TOOL_LENGTH` (0.12 m): length of the screwdriver bit from the flange. `TOOL_SIGN = -1` if the tool points the wrong way.

Known limits of the current code: IK ignores collisions (the arm could pass through the table), paths between waypoints are joint-space curves rather than straight lines, the `UNSCREW` step is just a timer, and the simulated friction of the real joints is ignored.

---

## 7. Roadmap

- [ ] Verify scripts 02-05 and update the status table
- [ ] Measure the real table and robot mounting position
- [ ] Collision-aware IK and straight-line tool paths
- [ ] Hole detection from an Intel RealSense L515 point cloud (camera frame -> table frame)
- [ ] Hardware interface: replace the simulated plant with a block that talks to the real Diana 7 (needs the vendor's API; start with slow position control through the vendor's safety layer, not raw torque control)

---

## 8. Working together on GitHub

- `main` should always run. Do not commit to it directly.
- One branch per piece of work: `git switch -c short-descriptive-name`.
- Small commits with messages that say what and why. Then `git push -u origin <branch>` and open a **pull request**; the other person reviews and merges.
- `git pull` before you start working each day.
- Do not commit generated outputs (`*.svg`, `*.png`, `__pycache__/`); they are in `.gitignore`.
