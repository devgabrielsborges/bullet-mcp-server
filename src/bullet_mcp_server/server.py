import asyncio
import json
import math
import random
import time

import pybullet as p
import pybullet_data
from mcp.server.models import InitializationOptions
import mcp.types as types
from mcp.server import NotificationOptions, Server
from pydantic import AnyUrl
import mcp.server.stdio

simulation_state = {
    "physics_client": None,
    "physics_client_id": None,
    "instance_counter": 0,
    "objects": {},  # object_id -> object_info
    "simulation_running": False,
    "gravity": [0, 0, -9.81],
    "time_step": 1.0 / 240.0,
    "real_time": True,
    "mode": "DIRECT",  # Track current connection mode
    "initialized": False,  # Track if client has been properly initialized
}

server = Server("bullet-mcp-server")


def ensure_physics_client():
    """
    Ensure PyBullet physics client is initialized and connected.
    This maintains a persistent connection across tool calls.
    """
    if simulation_state["physics_client"] is not None:
        # Check if the existing connection is still valid
        try:
            # Test the connection by querying the number of bodies
            p.getNumBodies(physicsClientId=simulation_state["physics_client_id"])
            return  # Connection is valid, nothing to do
        except Exception:
            # Connection is broken, mark as uninitialized
            simulation_state["physics_client"] = None
            simulation_state["physics_client_id"] = None
            simulation_state["initialized"] = False

    # Create new connection if needed
    if simulation_state["physics_client"] is None:
        import os

        if os.environ.get("DISPLAY"):
            try:
                client_id = p.connect(p.GUI)
                simulation_state["physics_client"] = True  # Mark as connected
                simulation_state["physics_client_id"] = client_id
                simulation_state["mode"] = "GUI"
                simulation_state["instance_counter"] += 1
                print("Connected to PyBullet in GUI mode")
            except Exception as e:
                print(f"GUI mode failed ({e}), falling back to DIRECT mode")
                try:
                    client_id = p.connect(p.DIRECT)
                    simulation_state["physics_client"] = True
                    simulation_state["physics_client_id"] = client_id
                    simulation_state["mode"] = "DIRECT"
                    simulation_state["instance_counter"] += 1
                    print("Connected to PyBullet in DIRECT mode")
                except Exception as e2:
                    print(f"Failed to connect in DIRECT mode: {e2}")
                    raise e2
        else:
            # No display available, use DIRECT mode
            try:
                client_id = p.connect(p.DIRECT)
                simulation_state["physics_client"] = True
                simulation_state["physics_client_id"] = client_id
                simulation_state["mode"] = "DIRECT"
                simulation_state["instance_counter"] += 1
                print("Connected to PyBullet in DIRECT mode (no display)")
            except Exception as e:
                print(f"Failed to connect in DIRECT mode: {e}")
                raise e

        # Initialize the physics world only once
        if not simulation_state["initialized"]:
            p.setAdditionalSearchPath(
                pybullet_data.getDataPath(),
                physicsClientId=simulation_state["physics_client_id"],
            )
            p.setGravity(
                *simulation_state["gravity"],
                physicsClientId=simulation_state["physics_client_id"],
            )
            p.setTimeStep(
                simulation_state["time_step"],
                physicsClientId=simulation_state["physics_client_id"],
            )
            p.setRealTimeSimulation(
                1 if simulation_state["real_time"] else 0,
                physicsClientId=simulation_state["physics_client_id"],
            )

            # Load default plane
            plane_id = p.loadURDF(
                "plane.urdf", physicsClientId=simulation_state["physics_client_id"]
            )
            simulation_state["objects"][plane_id] = {
                "name": "ground_plane",
                "type": "plane",
                "urdf": "plane.urdf",
                "position": [0, 0, 0],
                "orientation": [0, 0, 0, 1],
            }
            simulation_state["initialized"] = True


def get_physics_client_id():
    """Get the current physics client ID, ensuring client is initialized."""
    ensure_physics_client()
    return simulation_state["physics_client_id"]


def reset_simulation_state():
    """
    Reset the simulation state without disconnecting the physics client.
    This removes all objects except the ground plane and resets physics parameters.
    """
    ensure_physics_client()
    client_id = simulation_state["physics_client_id"]

    # Remove all objects except the ground plane
    objects_to_remove = []
    for obj_id, obj_info in simulation_state["objects"].items():
        if obj_info.get("name") != "ground_plane":
            objects_to_remove.append(obj_id)

    for obj_id in objects_to_remove:
        try:
            p.removeBody(obj_id, physicsClientId=client_id)
            del simulation_state["objects"][obj_id]
        except Exception:
            # Object might have already been removed
            pass

    # Reset physics parameters to defaults
    p.setGravity(*simulation_state["gravity"], physicsClientId=client_id)
    p.setTimeStep(simulation_state["time_step"], physicsClientId=client_id)
    p.setRealTimeSimulation(
        1 if simulation_state["real_time"] else 0, physicsClientId=client_id
    )


def disconnect_physics_client():
    """Safely disconnect the physics client and reset state."""
    if simulation_state["physics_client"] is not None:
        try:
            p.disconnect(physicsClientId=simulation_state["physics_client_id"])
        except Exception:
            pass  # Ignore errors during disconnect

        simulation_state["physics_client"] = None
        simulation_state["physics_client_id"] = None
        simulation_state["initialized"] = False
        simulation_state["objects"] = {}
        simulation_state["simulation_running"] = False


@server.list_resources()
async def handle_list_resources() -> list[types.Resource]:
    """
    List available PyBullet simulation resources.
    """
    ensure_physics_client()

    resources = [
        types.Resource(
            uri=AnyUrl("bullet://simulation/status"),
            name="Simulation Status",
            description="Current status of the physics simulation",
            mimeType="application/json",
        ),
        types.Resource(
            uri=AnyUrl("bullet://simulation/objects"),
            name="Simulation Objects",
            description="List of all objects in the simulation",
            mimeType="application/json",
        ),
        types.Resource(
            uri=AnyUrl("bullet://simulation/physics_params"),
            name="Physics Parameters",
            description="Current physics simulation parameters",
            mimeType="application/json",
        ),
    ]

    # Add individual object resources
    for obj_id, obj_info in simulation_state["objects"].items():
        resources.append(
            types.Resource(
                uri=AnyUrl(f"bullet://objects/{obj_id}"),
                name=f"Object: {obj_info.get('name', f'obj_{obj_id}')}",
                description=f"Details for {obj_info.get('type', 'unknown')} object",
                mimeType="application/json",
            )
        )

    return resources


@server.read_resource()
async def handle_read_resource(uri: AnyUrl) -> str:
    """
    Read specific PyBullet simulation resource content.
    """
    ensure_physics_client()

    if uri.scheme != "bullet":
        raise ValueError(f"Unsupported URI scheme: {uri.scheme}")

    path_parts = uri.path.strip("/").split("/")

    if len(path_parts) >= 2 and path_parts[0] == "simulation":
        if path_parts[1] == "status":
            return json.dumps(
                {
                    "running": simulation_state["simulation_running"],
                    "mode": simulation_state["mode"],
                    "objects_count": len(simulation_state["objects"]),
                    "gravity": simulation_state["gravity"],
                    "time_step": simulation_state["time_step"],
                    "real_time": simulation_state["real_time"],
                },
                indent=2,
            )
        elif path_parts[1] == "objects":
            objects_info = {}
            for obj_id, obj_info in simulation_state["objects"].items():
                pos, orn = p.getBasePositionAndOrientation(obj_id)
                objects_info[str(obj_id)] = {
                    **obj_info,
                    "current_position": list(pos),
                    "current_orientation": list(orn),
                }
            return json.dumps(objects_info, indent=2)
        elif path_parts[1] == "physics_params":
            return json.dumps(
                {
                    "gravity": simulation_state["gravity"],
                    "time_step": simulation_state["time_step"],
                    "real_time_simulation": simulation_state["real_time"],
                },
                indent=2,
            )

    elif len(path_parts) >= 2 and path_parts[0] == "objects":
        obj_id = int(path_parts[1])
        if obj_id in simulation_state["objects"]:
            obj_info = simulation_state["objects"][obj_id]
            pos, orn = p.getBasePositionAndOrientation(obj_id)
            lin_vel, ang_vel = p.getBaseVelocity(obj_id)

            return json.dumps(
                {
                    **obj_info,
                    "current_position": list(pos),
                    "current_orientation": list(orn),
                    "linear_velocity": list(lin_vel),
                    "angular_velocity": list(ang_vel),
                },
                indent=2,
            )
        else:
            raise ValueError(f"Object not found: {obj_id}")

    raise ValueError(f"Resource not found: {uri.path}")


@server.list_tools()
async def handle_list_tools() -> list[types.Tool]:
    """
    List available PyBullet simulation tools.
    """
    return [
        types.Tool(
            name="create_simulation",
            description="Initialize or configure the physics simulation (uses persistent connection)",
            inputSchema={
                "type": "object",
                "properties": {
                    "mode": {
                        "type": "string",
                        "enum": ["GUI", "DIRECT"],
                        "default": "DIRECT",
                    },
                    "gravity": {
                        "type": "array",
                        "items": {"type": "number"},
                        "default": [0, 0, -9.81],
                    },
                    "time_step": {"type": "number", "default": 0.004167},
                    "real_time": {"type": "boolean", "default": True},
                    "force_reconnect": {
                        "type": "boolean",
                        "default": False,
                        "description": "Force disconnect and reconnect the physics client",
                    },
                },
            },
        ),
        types.Tool(
            name="reset_simulation",
            description="Reset simulation state without disconnecting the physics client",
            inputSchema={
                "type": "object",
                "properties": {
                    "keep_objects": {
                        "type": "array",
                        "items": {"type": "string"},
                        "default": ["ground_plane"],
                        "description": "List of object names to keep (others will be removed)",
                    }
                },
            },
        ),
        types.Tool(
            name="load_object",
            description="Load an object into the simulation",
            inputSchema={
                "type": "object",
                "properties": {
                    "urdf_file": {
                        "type": "string",
                        "description": "URDF file name (e.g., 'cube.urdf', 'r2d2.urdf')",
                    },
                    "position": {
                        "type": "array",
                        "items": {"type": "number"},
                        "default": [0, 0, 1],
                    },
                    "orientation": {
                        "type": "array",
                        "items": {"type": "number"},
                        "default": [0, 0, 0, 1],
                    },
                    "name": {
                        "type": "string",
                        "description": "Optional name for the object",
                    },
                    "global_scaling": {"type": "number", "default": 1.0},
                },
                "required": ["urdf_file"],
            },
        ),
        types.Tool(
            name="set_object_color",
            description="Change the color of an object",
            inputSchema={
                "type": "object",
                "properties": {
                    "object_id": {"type": "integer"},
                    "color": {
                        "type": "array",
                        "items": {"type": "number"},
                        "minItems": 3,
                        "maxItems": 4,
                    },
                },
                "required": ["object_id", "color"],
            },
        ),
        types.Tool(
            name="apply_force",
            description="Apply force to an object",
            inputSchema={
                "type": "object",
                "properties": {
                    "object_id": {"type": "integer"},
                    "force": {
                        "type": "array",
                        "items": {"type": "number"},
                        "minItems": 3,
                        "maxItems": 3,
                    },
                    "position": {
                        "type": "array",
                        "items": {"type": "number"},
                        "default": [0, 0, 0],
                    },
                    "flags": {
                        "type": "string",
                        "enum": ["WORLD_FRAME", "LINK_FRAME"],
                        "default": "WORLD_FRAME",
                    },
                },
                "required": ["object_id", "force"],
            },
        ),
        types.Tool(
            name="set_velocity",
            description="Set linear and angular velocity of an object",
            inputSchema={
                "type": "object",
                "properties": {
                    "object_id": {"type": "integer"},
                    "linear_velocity": {
                        "type": "array",
                        "items": {"type": "number"},
                        "default": [0, 0, 0],
                    },
                    "angular_velocity": {
                        "type": "array",
                        "items": {"type": "number"},
                        "default": [0, 0, 0],
                    },
                },
                "required": ["object_id"],
            },
        ),
        types.Tool(
            name="step_simulation",
            description="Step the physics simulation forward",
            inputSchema={
                "type": "object",
                "properties": {"steps": {"type": "integer", "default": 1}},
            },
        ),
        types.Tool(
            name="get_object_info",
            description="Get detailed information about an object",
            inputSchema={
                "type": "object",
                "properties": {"object_id": {"type": "integer"}},
                "required": ["object_id"],
            },
        ),
        types.Tool(
            name="create_random_scene",
            description="Create a scene with random objects for testing",
            inputSchema={
                "type": "object",
                "properties": {
                    "num_objects": {
                        "type": "integer",
                        "default": 10,
                        "minimum": 1,
                        "maximum": 50,
                    },
                    "object_types": {
                        "type": "array",
                        "items": {"type": "string"},
                        "default": ["cube", "sphere"],
                    },
                    "area_size": {"type": "number", "default": 5.0},
                },
            },
        ),
        types.Tool(
            name="reset_object_pose",
            description="Reset an object's position and orientation",
            inputSchema={
                "type": "object",
                "properties": {
                    "object_id": {"type": "integer"},
                    "position": {
                        "type": "array",
                        "items": {"type": "number"},
                        "minItems": 3,
                        "maxItems": 3,
                    },
                    "orientation": {
                        "type": "array",
                        "items": {"type": "number"},
                        "minItems": 4,
                        "maxItems": 4,
                    },
                },
                "required": ["object_id", "position"],
            },
        ),
        types.Tool(
            name="create_quantitative_test_scene",
            description="Create a standardized scene for quantitative physics testing",
            inputSchema={
                "type": "object",
                "properties": {
                    "test_type": {
                        "type": "string",
                        "enum": [
                            "collision",
                            "friction",
                            "gravity",
                            "stability",
                            "pendulum",
                            "chain",
                            "tower",
                        ],
                        "description": "Type of quantitative test to set up",
                        "default": "collision",
                    },
                    "parameters": {
                        "type": "object",
                        "description": "Test-specific parameters",
                        "properties": {
                            "num_objects": {"type": "integer", "default": 5},
                            "mass": {"type": "number", "default": 1.0},
                            "friction": {"type": "number", "default": 0.5},
                            "restitution": {"type": "number", "default": 0.7},
                        },
                    },
                },
                "required": ["test_type"],
            },
        ),
        types.Tool(
            name="create_qualitative_test_scene",
            description="Create a visually diverse scene for qualitative testing and demonstration",
            inputSchema={
                "type": "object",
                "properties": {
                    "scene_type": {
                        "type": "string",
                        "enum": [
                            "showcase",
                            "stress_test",
                            "interactive",
                            "artistic",
                            "playground",
                        ],
                        "description": "Type of qualitative scene to create",
                        "default": "showcase",
                    },
                    "complexity": {
                        "type": "string",
                        "enum": ["low", "medium", "high"],
                        "description": "Scene complexity level",
                        "default": "medium",
                    },
                    "theme": {
                        "type": "string",
                        "enum": ["colorful", "realistic", "minimalist", "chaotic"],
                        "description": "Visual theme for the scene",
                        "default": "colorful",
                    },
                },
                "required": ["scene_type"],
            },
        ),
    ]


@server.call_tool()
async def handle_call_tool(
    name: str, arguments: dict | None
) -> list[types.TextContent | types.ImageContent | types.EmbeddedResource]:
    """
    Handle PyBullet simulation tool execution requests.
    """
    if not arguments:
        arguments = {}

    try:
        if name == "create_simulation":
            mode = arguments.get("mode", "DIRECT")
            gravity = arguments.get("gravity", [0, 0, -9.81])
            time_step = arguments.get("time_step", 1.0 / 240.0)
            real_time = arguments.get("real_time", True)
            force_reconnect = arguments.get("force_reconnect", False)

            simulation_state["gravity"] = gravity
            simulation_state["time_step"] = time_step
            simulation_state["real_time"] = real_time

            # Only disconnect if mode change is requested or force_reconnect is True
            need_reconnect = force_reconnect or (
                simulation_state["mode"] != mode
                and simulation_state["physics_client"] is not None
                and mode
                != "DIRECT"  # Don't reconnect if requesting DIRECT but currently in GUI
            )

            if need_reconnect:
                disconnect_physics_client()
                simulation_state["mode"] = mode
                # Force creation of new instance
                ensure_physics_client()
                connection_msg = (
                    f"Reconnected PyBullet in {simulation_state['mode']} mode"
                )
            else:
                ensure_physics_client()
                connection_msg = f"Using persistent PyBullet connection in {simulation_state['mode']} mode"

            client_id = get_physics_client_id()

            # Apply new physics parameters to existing connection
            p.setGravity(*gravity, physicsClientId=client_id)
            p.setTimeStep(time_step, physicsClientId=client_id)
            p.setRealTimeSimulation(1 if real_time else 0, physicsClientId=client_id)

            simulation_state["simulation_running"] = True

            return [
                types.TextContent(
                    type="text",
                    text=f"Physics simulation ready: {connection_msg}. Gravity: {gravity}",
                )
            ]

        elif name == "reset_simulation":
            ensure_physics_client()
            client_id = get_physics_client_id()

            keep_objects = arguments.get("keep_objects", ["ground_plane"])

            # Remove objects not in the keep list
            objects_to_remove = []
            for obj_id, obj_info in simulation_state["objects"].items():
                if obj_info.get("name") not in keep_objects:
                    objects_to_remove.append(obj_id)

            removed_count = 0
            for obj_id in objects_to_remove:
                try:
                    p.removeBody(obj_id, physicsClientId=client_id)
                    del simulation_state["objects"][obj_id]
                    removed_count += 1
                except Exception:
                    # Object might have already been removed
                    pass

            # Reset physics parameters to current state values
            p.setGravity(*simulation_state["gravity"], physicsClientId=client_id)
            p.setTimeStep(simulation_state["time_step"], physicsClientId=client_id)
            p.setRealTimeSimulation(
                1 if simulation_state["real_time"] else 0, physicsClientId=client_id
            )

            remaining_objects = list(simulation_state["objects"].keys())

            return [
                types.TextContent(
                    type="text",
                    text=f"Simulation reset: removed {removed_count} objects, "
                    f"kept {len(remaining_objects)} objects. "
                    f"Physics client remains connected.",
                )
            ]

        elif name == "load_object":
            ensure_physics_client()
            client_id = get_physics_client_id()

            urdf_file = arguments["urdf_file"]
            position = arguments.get("position", [0, 0, 1])
            orientation = arguments.get("orientation", [0, 0, 0, 1])
            obj_name = arguments.get("name", f"obj_{len(simulation_state['objects'])}")
            scaling = arguments.get("global_scaling", 1.0)

            # Convert euler to quaternion if needed
            if len(orientation) == 3:
                orientation = p.getQuaternionFromEuler(orientation)

            obj_id = p.loadURDF(
                urdf_file,
                position,
                orientation,
                globalScaling=scaling,
                physicsClientId=client_id,
            )

            simulation_state["objects"][obj_id] = {
                "name": obj_name,
                "type": urdf_file.split(".")[0],
                "urdf": urdf_file,
                "position": position,
                "orientation": orientation,
                "scaling": scaling,
            }

            return [
                types.TextContent(
                    type="text",
                    text=f"Loaded {urdf_file} as object {obj_id} at position {position}",
                )
            ]

        elif name == "set_object_color":
            ensure_physics_client()

            obj_id = arguments["object_id"]
            color = arguments["color"]

            if len(color) == 3:
                color.append(1.0)  # Add alpha if not provided

            p.changeVisualShape(obj_id, -1, rgbaColor=color)

            return [
                types.TextContent(
                    type="text", text=f"Set color of object {obj_id} to {color}"
                )
            ]

        elif name == "apply_force":
            ensure_physics_client()

            obj_id = arguments["object_id"]
            force = arguments["force"]
            position = arguments.get("position", [0, 0, 0])
            flags = arguments.get("flags", "WORLD_FRAME")

            flag_value = p.WORLD_FRAME if flags == "WORLD_FRAME" else p.LINK_FRAME
            p.applyExternalForce(obj_id, -1, force, position, flag_value)

            return [
                types.TextContent(
                    type="text",
                    text=f"Applied force {force} to object {obj_id} at position {position}",
                )
            ]

        elif name == "set_velocity":
            ensure_physics_client()

            obj_id = arguments["object_id"]
            lin_vel = arguments.get("linear_velocity", [0, 0, 0])
            ang_vel = arguments.get("angular_velocity", [0, 0, 0])

            p.resetBaseVelocity(obj_id, lin_vel, ang_vel)

            return [
                types.TextContent(
                    type="text",
                    text=f"Set velocity of object {obj_id}: linear={lin_vel}, angular={ang_vel}",
                )
            ]

        elif name == "step_simulation":
            ensure_physics_client()

            steps = arguments.get("steps", 1)

            for _ in range(steps):
                p.stepSimulation()
                if simulation_state["real_time"]:
                    time.sleep(simulation_state["time_step"])

            return [
                types.TextContent(
                    type="text", text=f"Stepped simulation {steps} step(s)"
                )
            ]

        elif name == "get_object_info":
            ensure_physics_client()

            obj_id = arguments["object_id"]

            if obj_id not in simulation_state["objects"]:
                raise ValueError(f"Object {obj_id} not found")

            pos, orn = p.getBasePositionAndOrientation(obj_id)
            lin_vel, ang_vel = p.getBaseVelocity(obj_id)
            obj_info = simulation_state["objects"][obj_id]

            info = {
                **obj_info,
                "current_position": list(pos),
                "current_orientation": list(orn),
                "linear_velocity": list(lin_vel),
                "angular_velocity": list(ang_vel),
            }

            return [
                types.TextContent(
                    type="text",
                    text=f"Object {obj_id} info:\n{json.dumps(info, indent=2)}",
                )
            ]

        elif name == "create_random_scene":
            ensure_physics_client()

            num_objects = arguments.get("num_objects", 10)
            object_types = arguments.get("object_types", ["cube", "sphere"])
            area_size = arguments.get("area_size", 5.0)

            created_objects = []

            for i in range(num_objects):
                obj_type = random.choice(object_types)
                urdf_file = (
                    f"{obj_type}_small.urdf"
                    if obj_type in ["cube", "sphere"]
                    else f"{obj_type}.urdf"
                )

                # Random position
                x = random.uniform(-area_size / 2, area_size / 2)
                y = random.uniform(-area_size / 2, area_size / 2)
                z = random.uniform(1, area_size)
                position = [x, y, z]

                # Random orientation
                orientation = p.getQuaternionFromEuler(
                    [
                        random.uniform(0, 2 * math.pi),
                        random.uniform(0, 2 * math.pi),
                        random.uniform(0, 2 * math.pi),
                    ]
                )

                try:
                    obj_id = p.loadURDF(urdf_file, position, orientation)

                    # Random color
                    color = [random.random(), random.random(), random.random(), 1.0]
                    p.changeVisualShape(obj_id, -1, rgbaColor=color)

                    simulation_state["objects"][obj_id] = {
                        "name": f"random_{obj_type}_{i}",
                        "type": obj_type,
                        "urdf": urdf_file,
                        "position": position,
                        "orientation": list(orientation),
                    }

                    created_objects.append(obj_id)

                except Exception as e:
                    # Skip objects that fail to load
                    pass

            return [
                types.TextContent(
                    type="text",
                    text=f"Created random scene with {len(created_objects)} objects: {created_objects}",
                )
            ]

        elif name == "reset_object_pose":
            ensure_physics_client()

            obj_id = arguments["object_id"]
            position = arguments["position"]
            orientation = arguments.get("orientation", [0, 0, 0, 1])

            if len(orientation) == 3:
                orientation = p.getQuaternionFromEuler(orientation)

            p.resetBasePositionAndOrientation(obj_id, position, orientation)

            # Update stored info
            if obj_id in simulation_state["objects"]:
                simulation_state["objects"][obj_id]["position"] = position
                simulation_state["objects"][obj_id]["orientation"] = list(orientation)

            return [
                types.TextContent(
                    type="text",
                    text=f"Reset object {obj_id} to position {position}, orientation {orientation}",
                )
            ]

        elif name == "create_quantitative_test_scene":
            ensure_physics_client()

            test_type = arguments["test_type"]
            params = arguments.get("parameters", {})

            created_objects = []
            test_info = {}

            if test_type == "collision":
                sphere1_id = p.loadURDF("sphere_small.urdf", [0, -2, 1], [0, 0, 0, 1])
                sphere2_id = p.loadURDF("sphere_small.urdf", [0, 2, 1], [0, 0, 0, 1])

                p.resetBaseVelocity(sphere1_id, [0, 5, 0], [0, 0, 0])
                p.resetBaseVelocity(sphere2_id, [0, -5, 0], [0, 0, 0])

                p.changeVisualShape(sphere1_id, -1, rgbaColor=[1, 0, 0, 1])  # Red
                p.changeVisualShape(sphere2_id, -1, rgbaColor=[0, 0, 1, 1])  # Blue

                created_objects = [sphere1_id, sphere2_id]
                test_info = {
                    "test_type": "collision",
                    "description": "Two spheres colliding with measurable velocities",
                    "expected_outcome": "Conservation of momentum",
                    "measurement_points": [
                        "before_collision",
                        "at_collision",
                        "after_collision",
                    ],
                }

            elif test_type == "friction":
                incline_id = p.loadURDF(
                    "cube.urdf",
                    [0, 0, 0.1],
                    p.getQuaternionFromEuler([0, math.pi / 6, 0]),
                    globalScaling=5.0,
                )

                friction_values = [0.1, 0.5, 1.0]
                for i, friction in enumerate(friction_values):
                    obj_id = p.loadURDF("cube_small.urdf", [-3 + i, 0, 2], [0, 0, 0, 1])
                    p.changeDynamics(obj_id, -1, lateralFriction=friction)

                    colors = [[1, 0, 0, 1], [1, 1, 0, 1], [0, 1, 0, 1]]
                    p.changeVisualShape(obj_id, -1, rgbaColor=colors[i])
                    created_objects.append(obj_id)

                test_info = {
                    "test_type": "friction",
                    "description": "Objects with different friction coefficients on inclined plane",
                    "friction_values": friction_values,
                    "expected_outcome": "Different sliding speeds based on friction",
                    "incline_angle": "30 degrees",
                }

            elif test_type == "gravity":
                masses = [0.5, 1.0, 2.0, 5.0]
                for i, mass in enumerate(masses):
                    obj_id = p.loadURDF(
                        "sphere_small.urdf", [i * 0.5, 0, 5], [0, 0, 0, 1]
                    )
                    p.changeDynamics(obj_id, -1, mass=mass)

                    intensity = 0.2 + 0.8 * (i / len(masses))
                    p.changeVisualShape(
                        obj_id, -1, rgbaColor=[intensity, intensity, intensity, 1]
                    )
                    created_objects.append(obj_id)

                test_info = {
                    "test_type": "gravity",
                    "description": "Free fall test with different masses",
                    "masses": masses,
                    "expected_outcome": "All objects fall at same rate (ignoring air resistance)",
                    "drop_height": 5.0,
                }

            elif test_type == "stability":
                # Tower building test for stability analysis
                num_blocks = params.get("num_objects", 5)
                for i in range(num_blocks):
                    obj_id = p.loadURDF(
                        "cube_small.urdf", [0, 0, 0.5 + i * 1.0], [0, 0, 0, 1]
                    )

                    # Slight random offset to test stability
                    offset = random.uniform(-0.1, 0.1)
                    p.resetBasePositionAndOrientation(
                        obj_id, [offset, 0, 0.5 + i * 1.0], [0, 0, 0, 1]
                    )

                    # Alternate colors
                    color = [1, 0, 0, 1] if i % 2 == 0 else [0, 0, 1, 1]
                    p.changeVisualShape(obj_id, -1, rgbaColor=color)
                    created_objects.append(obj_id)

                test_info = {
                    "test_type": "stability",
                    "description": "Tower stability test with stacked blocks",
                    "num_blocks": num_blocks,
                    "expected_outcome": "Tower may collapse based on positioning accuracy",
                }

            elif test_type == "pendulum":
                # Simple pendulum for oscillation testing
                # Create anchor point
                anchor_id = p.loadURDF("cube_small.urdf", [0, 0, 3], [0, 0, 0, 1])
                p.changeDynamics(anchor_id, -1, mass=0)  # Make it static

                # Create pendulum bob
                bob_id = p.loadURDF("sphere_small.urdf", [1.5, 0, 1.5], [0, 0, 0, 1])

                # Create constraint (rope)
                constraint_id = p.createConstraint(
                    anchor_id,
                    -1,
                    bob_id,
                    -1,
                    p.JOINT_POINT2POINT,
                    [0, 0, 0],
                    [0, 0, 0],
                    [0, 0, 0],
                )

                # Set colors
                p.changeVisualShape(
                    anchor_id, -1, rgbaColor=[0.5, 0.5, 0.5, 1]
                )  # Gray anchor
                p.changeVisualShape(bob_id, -1, rgbaColor=[1, 0, 0, 1])  # Red bob

                created_objects = [anchor_id, bob_id]
                test_info = {
                    "test_type": "pendulum",
                    "description": "Simple pendulum for oscillation period measurement",
                    "pendulum_length": 1.5,
                    "expected_outcome": "Periodic oscillation",
                    "constraint_id": constraint_id,
                }

            # Store test objects and info
            for obj_id in created_objects:
                simulation_state["objects"][obj_id] = {
                    "name": f"{test_type}_test_object_{obj_id}",
                    "type": test_type,
                    "test_info": test_info,
                }

            return [
                types.TextContent(
                    type="text",
                    text=f"Created quantitative test scene: {test_type}\n"
                    f"Objects created: {created_objects}\n"
                    f"Test info: {json.dumps(test_info, indent=2)}",
                )
            ]

        elif name == "create_qualitative_test_scene":
            ensure_physics_client()

            scene_type = arguments["scene_type"]
            complexity = arguments.get("complexity", "medium")
            theme = arguments.get("theme", "colorful")

            created_objects = []
            scene_info = {}

            # Define complexity parameters
            complexity_params = {
                "low": {"num_objects": 5, "area_size": 3.0, "max_height": 2.0},
                "medium": {"num_objects": 15, "area_size": 5.0, "max_height": 4.0},
                "high": {"num_objects": 30, "area_size": 8.0, "max_height": 6.0},
            }

            params = complexity_params[complexity]

            if scene_type == "showcase":
                # Organized display of different object types
                object_types = [
                    "cube_small.urdf",
                    "sphere_small.urdf",
                    "duck_vhacd.urdf",
                    "teddy_vhacd.urdf",
                ]
                positions = [[0, -2, 1], [0, 0, 1], [0, 2, 1], [2, 0, 1]]

                for i, (obj_type, pos) in enumerate(zip(object_types, positions)):
                    try:
                        obj_id = p.loadURDF(obj_type, pos, [0, 0, 0, 1])

                        if theme == "colorful":
                            colors = [
                                [1, 0, 0, 1],
                                [0, 1, 0, 1],
                                [0, 0, 1, 1],
                                [1, 1, 0, 1],
                            ]
                            p.changeVisualShape(obj_id, -1, rgbaColor=colors[i])
                        elif theme == "realistic":
                            # Use natural colors
                            colors = [
                                [0.8, 0.6, 0.4, 1],
                                [0.7, 0.7, 0.7, 1],
                                [1, 1, 0, 1],
                                [0.6, 0.3, 0.1, 1],
                            ]
                            p.changeVisualShape(obj_id, -1, rgbaColor=colors[i])

                        created_objects.append(obj_id)
                    except:
                        pass  # Skip objects that fail to load

            elif scene_type == "stress_test":
                # Many objects for performance testing
                for i in range(params["num_objects"]):
                    obj_type = random.choice(["cube_small.urdf", "sphere_small.urdf"])

                    # Random positions in a grid-like pattern
                    x = (i % 6) * 1.0 - 2.5
                    y = (i // 6) * 1.0 - 2.5
                    z = random.uniform(1, params["max_height"])

                    try:
                        obj_id = p.loadURDF(
                            obj_type,
                            [x, y, z],
                            p.getQuaternionFromEuler(
                                [
                                    random.uniform(0, 2 * math.pi),
                                    random.uniform(0, 2 * math.pi),
                                    random.uniform(0, 2 * math.pi),
                                ]
                            ),
                        )

                        if theme == "colorful":
                            color = [
                                random.random(),
                                random.random(),
                                random.random(),
                                1.0,
                            ]
                        elif theme == "chaotic":
                            # High contrast colors
                            color = [
                                random.choice([0, 1]),
                                random.choice([0, 1]),
                                random.choice([0, 1]),
                                1.0,
                            ]
                        else:
                            # Monochrome
                            intensity = random.uniform(0.2, 0.8)
                            color = [intensity, intensity, intensity, 1.0]

                        p.changeVisualShape(obj_id, -1, rgbaColor=color)
                        created_objects.append(obj_id)
                    except:
                        pass

            elif scene_type == "interactive":
                # Scene designed for user interaction
                # Central large object
                center_id = p.loadURDF(
                    "cube.urdf", [0, 0, 1], [0, 0, 0, 1], globalScaling=2.0
                )
                p.changeVisualShape(center_id, -1, rgbaColor=[0.5, 0.5, 1.0, 1])
                created_objects.append(center_id)

                # Surrounding smaller objects in a circle
                num_surrounding = 8
                radius = 3.0
                for i in range(num_surrounding):
                    angle = (2 * math.pi * i) / num_surrounding
                    x = radius * math.cos(angle)
                    y = radius * math.sin(angle)

                    obj_id = p.loadURDF("sphere_small.urdf", [x, y, 0.5], [0, 0, 0, 1])

                    # Color based on position
                    if theme == "colorful":
                        hue = i / num_surrounding
                        color = [hue, 1.0, 1.0 - hue, 1.0]
                    else:
                        color = [0.8, 0.8, 0.8, 1.0]

                    p.changeVisualShape(obj_id, -1, rgbaColor=color)
                    created_objects.append(obj_id)

            # Store scene objects and info
            scene_info = {
                "scene_type": scene_type,
                "complexity": complexity,
                "theme": theme,
                "num_objects": len(created_objects),
                "description": f"Qualitative test scene: {scene_type} with {complexity} complexity",
            }

            for obj_id in created_objects:
                simulation_state["objects"][obj_id] = {
                    "name": f"{scene_type}_{complexity}_object_{obj_id}",
                    "type": scene_type,
                    "scene_info": scene_info,
                }

            return [
                types.TextContent(
                    type="text",
                    text=f"Created qualitative test scene: {scene_type}\n"
                    f"Complexity: {complexity}, Theme: {theme}\n"
                    f"Objects created: {len(created_objects)}\n"
                    f"Scene info: {json.dumps(scene_info, indent=2)}",
                )
            ]

        else:
            raise ValueError(f"Unknown tool: {name}")

    except Exception as e:
        return [
            types.TextContent(type="text", text=f"Error executing {name}: {str(e)}")
        ]


async def main():
    # Run the server using stdin/stdout streams
    async with mcp.server.stdio.stdio_server() as (read_stream, write_stream):
        await server.run(
            read_stream,
            write_stream,
            InitializationOptions(
                server_name="bullet-mcp-server",
                server_version="0.1.0",
                capabilities=server.get_capabilities(
                    notification_options=NotificationOptions(),
                    experimental_capabilities={},
                ),
            ),
        )


if __name__ == "__main__":
    asyncio.run(main())
