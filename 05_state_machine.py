"""
Step 5: the unscrewing sequence as a STATE MACHINE written as our own Drake LeafSystem.

Difference to step 4:
  Step 4:  TrajectorySource replays a schedule fixed in advance ("at 3 s be at X").
  Step 5:  UnscrewStateMachine DECIDES while running. It looks at the measured joint angles
           and only moves on to the next step when the robot has really arrived. If it does
           not arrive in time it goes to FAULT and holds still.

Diagram:
    plant --state q,v--> UnscrewStateMachine --desired q,v--> InverseDynamicsController
      ^                                                              |
      +----------------------------torques--------------------------+

Run:
    ~/drake-env/bin/pip install matplotlib pydot      (once; matplotlib only for the plot)
    ~/drake-env/bin/python scripts/05_state_machine.py 2>&1 | grep -v WARNING
"""
import os
import sys
from dataclasses import dataclass
import numpy as np
from pydrake.all import (
    DiagramBuilder, AddMultibodyPlantSceneGraph, MultibodyPlant, StartMeshcat,
    MeshcatVisualizer, Simulator, InverseDynamicsController, LeafSystem, LogVectorOutput,
    InverseKinematics, Solve, RigidTransform, RotationMatrix, Cylinder, Rgba,
)

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, os.path.join(ROOT, "src"))
from scene import add_table, add_diana7, add_object  # noqa: E402

TABLE_URDF = os.path.join(ROOT, "models", "table", "welding_table.urdf")
ROBOT_URDF = os.path.join(ROOT, "models", "diana7", "diana7.urdf")
OBJECT_URDF = os.path.join(ROOT, "models", "object", "plate_with_holes.urdf")

# =========================== PARAMETERS (same meaning as in step 4) ======================
TABLE_TOP_HEIGHT = 0.9
ROBOT_XY, ROBOT_YAW_DEG = (0.0, 0.0), 0.0
OBJECT_XY, OBJECT_YAW_DEG = (0.50, 0.0), 0.0
OBJECT_THICKNESS = 0.02                # must equal the box height in plate_with_holes.urdf
HOLES_IN_OBJECT = [(-0.06, -0.035), (0.06, -0.035), (0.06, 0.035), (-0.06, 0.035)]
HOLE_RADIUS = 0.004
TOOL_LENGTH = 0.12                     # [m] flange -> screwdriver tip (placeholder)
TOOL_SIGN = 1.0                        # -1.0 if the red tool points the wrong way
APPROACH_HEIGHT = 0.05
T_TRAVEL, T_Z, T_SCREW = 2.0, 1.0, 1.0  # nominal durations of the steps [s]
Q_HOME = np.array([0.0, 0.5, 0.0, -1.0, 0.0, 0.5, 0.0])

ARRIVE_TOL = 0.01      # [rad] "arrived" = every joint within this of the goal
ARRIVE_TIMEOUT = 2.0   # [s] extra time allowed to arrive after the nominal duration, else FAULT
KP = 300.0             # controller stiffness. Try 5.0 to see the machine go to FAULT.
# =========================================================================================

FAULT = -1             # special state index; DONE is index len(steps)


@dataclass
class Step:
    """One state of the machine: move the joints to q_goal, nominally in `duration` seconds."""
    name: str
    q_goal: np.ndarray
    duration: float


class UnscrewStateMachine(LeafSystem):
    """Our own Drake block.  Ports:
         input  "estimated_state" (2*nq): measured [q; v] from the plant
         output "desired_state"   (2*nq): [q_des; v_des] for the controller
         output "step_index"      (1)   : current state number, for logging/plotting

    Discrete state (the machine's memory, 2 + nq numbers):
         [ current step index, time the step started, joint angles at the step start ]
    """

    def __init__(self, steps, q_home, period=0.005):
        super().__init__()
        self.steps = steps
        self.nq = len(q_home)
        self.done_index = len(steps)

        # Input: the measurement. On the real robot this wire would come from the encoders.
        self._state_in = self.DeclareVectorInputPort("estimated_state", 2 * self.nq)

        # Memory of the machine. Initial value: step 0, started at t=0, starting from home.
        self.DeclareDiscreteState(np.concatenate([[0.0, 0.0], q_home]))

        # "Think" every `period` seconds: this is where transitions are decided.
        self.DeclarePeriodicDiscreteUpdateEvent(period, 0.0, self._update)

        # Outputs: what the current state wants the robot to do.
        self.DeclareVectorOutputPort("desired_state", 2 * self.nq, self._calc_desired_state)
        self.DeclareVectorOutputPort("step_index", 1, self._calc_step_index)

    # ---- helpers ----
    def _read_memory(self, context):
        s = context.get_discrete_state(0).CopyToVector()
        return int(round(s[0])), s[1], s[2:2 + self.nq]

    def state_name(self, idx):
        if idx == FAULT:
            return "FAULT"
        if idx >= self.done_index:
            return "DONE"
        return self.steps[idx].name

    # ---- the transition logic ("which state next?") ----
    def _update(self, context, discrete_state):
        idx, t_start, q_start = self._read_memory(context)
        if idx == FAULT or idx >= self.done_index:
            return                                   # DONE and FAULT are final: do nothing
        t = context.get_time()
        elapsed = t - t_start
        step = self.steps[idx]
        q_meas = self._state_in.Eval(context)[:self.nq]

        if elapsed < step.duration:
            return                                   # still in the planned motion time

        error = np.max(np.abs(q_meas - step.q_goal))
        if error < ARRIVE_TOL:
            # CONDITION MET -> next state. The next motion starts from this step's goal,
            # so the reference never jumps. (For UNSCREW this is where a real "screw is
            # out" sensor check would replace the timer.)
            discrete_state.set_value(0, np.concatenate([[idx + 1, t], step.q_goal]))
            # print() inside an update is a side effect: fine for a demo log, not for
            # real code.
            print(f"t={t:6.2f} s   {step.name:18s} -> {self.state_name(idx + 1)}")
        elif elapsed > step.duration + ARRIVE_TIMEOUT:
            # NOT arrived in time -> FAULT: hold where the robot is right now.
            discrete_state.set_value(0, np.concatenate([[FAULT, t], q_meas]))
            print(f"t={t:6.2f} s   {step.name:18s} -> FAULT (error {error:.3f} rad)")

    # ---- what each state outputs ----
    def _calc_desired_state(self, context, output):
        idx, t_start, q_start = self._read_memory(context)
        nq = self.nq
        if idx == FAULT or idx >= self.done_index:
            # Hold still at the last known pose (goal of the last step, or pose at fault).
            output.SetFromVector(np.concatenate([q_start, np.zeros(nq)]))
            return
        step = self.steps[idx]
        tau = np.clip((context.get_time() - t_start) / step.duration, 0.0, 1.0)
        # Minimum-jerk time scaling: smooth start and stop (zero velocity at both ends).
        s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
        ds = (30 * tau**2 - 60 * tau**3 + 30 * tau**4) / step.duration
        delta = step.q_goal - q_start
        output.SetFromVector(np.concatenate([q_start + delta * s, delta * ds]))

    def _calc_step_index(self, context, output):
        output.SetFromVector([self._read_memory(context)[0]])


# =============================== Scene, IK (copied from step 4) ==========================
def build_scene(plant, tool_visual=False):
    _, table_frame = add_table(plant, TABLE_URDF, RigidTransform([0, 0, TABLE_TOP_HEIGHT]))
    X_TB = RigidTransform(RotationMatrix.MakeZRotation(np.radians(ROBOT_YAW_DEG)),
                          [ROBOT_XY[0], ROBOT_XY[1], 0.0])
    robot = add_diana7(plant, ROBOT_URDF, table_frame, X_TB)
    X_TO = RigidTransform(RotationMatrix.MakeZRotation(np.radians(OBJECT_YAW_DEG)),
                          [OBJECT_XY[0], OBJECT_XY[1], OBJECT_THICKNESS])
    _, object_frame = add_object(plant, OBJECT_URDF, table_frame, X_TO)
    if tool_visual:
        flange_body = plant.GetBodyByName("diana7_link8", robot)
        plant.RegisterVisualGeometry(flange_body,
                                     RigidTransform([0, 0, TOOL_SIGN * TOOL_LENGTH / 2]),
                                     Cylinder(0.006, TOOL_LENGTH), "tool",
                                     np.array([1.0, 0.1, 0.1, 1.0]))
    plant.Finalize()
    return robot, object_frame


ik_plant = MultibodyPlant(0.0)
ik_robot, ik_object_frame = build_scene(ik_plant)
ik_context = ik_plant.CreateDefaultContext()
world = ik_plant.world_frame()
flange = ik_plant.GetFrameByName("diana7_link8", ik_robot)
lower, upper = ik_plant.GetPositionLowerLimits(), ik_plant.GetPositionUpperLimits()
rng = np.random.default_rng(0)
X_WO = ik_plant.CalcRelativeTransform(ik_context, world, ik_object_frame)
holes_W = [X_WO @ np.array([x, y, 0.0]) for (x, y) in HOLES_IN_OBJECT]
tool_in_flange = np.array([0.0, 0.0, TOOL_SIGN * TOOL_LENGTH])


def solve_ik(p_target_W, q_guess):
    """Joint angles putting the tool tip at p_target_W, tool pointing straight down."""
    for guess in [q_guess] + [rng.uniform(lower, upper) for _ in range(30)]:
        ik = InverseKinematics(ik_plant, ik_context, with_joint_limits=True)
        ik.AddPositionConstraint(flange, tool_in_flange, world,
                                 p_target_W - 1e-4, p_target_W + 1e-4)
        ik.AddAngleBetweenVectorsConstraint(world, np.array([0.0, 0.0, -1.0]),
                                            flange, np.array([0.0, 0.0, TOOL_SIGN]),
                                            0.0, np.radians(3.0))
        ik.get_mutable_prog().SetInitialGuess(ik.q(), guess)
        result = Solve(ik.prog())
        if result.is_success():
            return result.GetSolution(ik.q())
    return None


# The list of states. Unlike step 4 there are no times here, only goals and nominal durations.
steps, q_prev = [], Q_HOME
for i, p_hole in enumerate(holes_W):
    q_above = solve_ik(p_hole + np.array([0, 0, APPROACH_HEIGHT]), q_prev)
    q_down = None if q_above is None else solve_ik(p_hole, q_above)
    if q_above is None or q_down is None:
        sys.exit(f"IK failed for hole {i + 1}. Move OBJECT_XY, flip TOOL_SIGN or shorten TOOL_LENGTH.")
    n = i + 1
    steps += [Step(f"APPROACH_HOLE_{n}", q_above, T_TRAVEL),
              Step(f"DESCEND_HOLE_{n}", q_down, T_Z),
              Step(f"UNSCREW_HOLE_{n}", q_down, T_SCREW),
              Step(f"ASCEND_HOLE_{n}", q_above, T_Z)]
    q_prev = q_above
steps.append(Step("RETURN_HOME", Q_HOME, T_TRAVEL))

# =============================== Diagram ================================================
meshcat = StartMeshcat()
builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-3)
robot, object_frame = build_scene(plant, tool_visual=True)
MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

control_plant = MultibodyPlant(0.0)
add_diana7(control_plant, ROBOT_URDF, control_plant.world_frame(), RigidTransform())
control_plant.Finalize()
nq = plant.num_positions(robot)
kp = np.full(nq, KP)
controller = builder.AddSystem(
    InverseDynamicsController(control_plant, kp, np.zeros(nq), 2.0 * np.sqrt(kp), False))

# OUR block goes into the diagram exactly like a built-in one.
machine = builder.AddSystem(UnscrewStateMachine(steps, Q_HOME))

builder.Connect(plant.get_state_output_port(robot), controller.get_input_port_estimated_state())
builder.Connect(controller.get_output_port_control(), plant.get_actuation_input_port(robot))
builder.Connect(plant.get_state_output_port(robot), machine.GetInputPort("estimated_state"))  # measurement -> machine
builder.Connect(machine.GetOutputPort("desired_state"), controller.get_input_port_desired_state())  # decision -> controller
logger = LogVectorOutput(machine.GetOutputPort("step_index"), builder, 0.01)  # for the timeline plot

diagram = builder.Build()

# Drake's own picture of the diagram (needs: pip install pydot, sudo apt install graphviz).
try:
    import pydot
    pydot.graph_from_dot_data(diagram.GetGraphvizString(max_depth=1))[0].write_svg(
        "diana7_state_machine_diagram.svg")
    print("Wrote diana7_state_machine_diagram.svg")
except Exception as e:
    print("Skipping diagram:", e)

context = diagram.CreateDefaultContext()
plant_context = plant.GetMyMutableContextFromRoot(context)
plant.SetPositions(plant_context, robot, Q_HOME)

X_WO_sim = plant.CalcRelativeTransform(plant_context, plant.world_frame(), object_frame)
for i, (x, y) in enumerate(HOLES_IN_OBJECT):
    meshcat.SetObject(f"holes/{i}", Cylinder(HOLE_RADIUS, 0.0006), Rgba(0.02, 0.02, 0.02, 1.0))
    meshcat.SetTransform(f"holes/{i}", X_WO_sim @ RigidTransform([x, y, 0.0004]))

# =============================== Run ====================================================
sim = Simulator(diagram, context)
machine_context = machine.GetMyContextFromRoot(context)
t_max = sum(s.duration + ARRIVE_TIMEOUT for s in steps)
meshcat.StartRecording()
while context.get_time() < t_max:
    sim.AdvanceTo(context.get_time() + 0.1)
    idx = int(round(machine_context.get_discrete_state(0).CopyToVector()[0]))
    if idx == FAULT or idx >= machine.done_index:
        break                                        # final state reached: stop the run
sim.AdvanceTo(context.get_time() + 1.0)              # keep recording a second of "holding"
meshcat.PublishRecording()
print(f"Finished in state {machine.state_name(int(round(machine_context.get_discrete_state(0).CopyToVector()[0])))} at t = {context.get_time():.1f} s")

# Timeline of the state machine: which state was active when. Good figure for a report.
try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    log = logger.FindLog(context)
    t, idx_log = log.sample_times(), log.data()[0]
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.step(t, idx_log, where="post")
    ticks = list(range(machine.done_index + 1))
    ax.set_yticks(ticks)
    ax.set_yticklabels([machine.state_name(k) for k in ticks], fontsize=7)
    ax.set_xlabel("time [s]")
    ax.set_title("State machine timeline (-1 = FAULT)")
    fig.tight_layout()
    fig.savefig("state_timeline.png", dpi=150)
    print("Wrote state_timeline.png")
except Exception as e:
    print("Skipping timeline plot:", e)

input("Meshcat is live (open the URL above). Press Enter to quit.")
