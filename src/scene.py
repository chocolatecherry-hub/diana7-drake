"""
Reusable helpers that build the lab scene inside a Drake MultibodyPlant.

Kept in one place so every script (view, controller, ...) builds the SAME scene.
Both functions only ADD things; the caller must call plant.Finalize() afterwards.
"""
from pydrake.all import Parser, RigidTransform


def add_table(plant, urdf_path, X_WT=RigidTransform()):
    """Add the welding table and fix it to the world.

    X_WT = pose of the table frame "table_top" expressed in the world frame W.
    Returns (model_instance, table_frame).
    """
    table = Parser(plant).AddModels(urdf_path)[0]
    table_frame = plant.GetFrameByName("table_top", table)
    # WeldFrames(frame_on_parent, frame_on_child, X_parent_child): "glue" the two frames
    # together with a fixed relative pose. Here: world <-> table_top.
    plant.WeldFrames(plant.world_frame(), table_frame, X_WT)
    return table, table_frame


def add_diana7(plant, urdf_path, parent_frame, X_PB):
    """Add the Diana 7 and weld its base to `parent_frame` (e.g. the table frame).

    X_PB = pose of the robot's "base" frame B expressed in the parent frame P.
    Also adds one actuator per revolute joint so a controller can be attached later.
    """
    robot = Parser(plant).AddModels(urdf_path)[0]
    base_frame = plant.GetFrameByName("base", robot)
    plant.WeldFrames(parent_frame, base_frame, X_PB)
    for j in plant.GetJointIndices(robot):
        joint = plant.get_joint(j)
        if joint.type_name() == "revolute":
            plant.AddJointActuator(f"act_{joint.name()}", joint)
    return robot


def add_object(plant, urdf_path, parent_frame, X_PO):
    """Add a work piece (e.g. the plate with screws) and weld it to `parent_frame`.

    X_PO = pose of the object frame "object_top" expressed in the parent frame P
    (here: the table frame). Returns (model_instance, object_frame).
    """
    obj = Parser(plant).AddModels(urdf_path)[0]
    object_frame = plant.GetFrameByName("object_top", obj)
    plant.WeldFrames(parent_frame, object_frame, X_PO)
    return obj, object_frame
