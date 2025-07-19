from isaacsim.simulation_app import SimulationApp
simulation_app = SimulationApp({"headless": False})

from isaacsim.core.api.world import World
from isaacsim.core.utils.prims import create_prim
from pxr import Gf, UsdPhysics, PhysxSchema, UsdGeom

def setup_dummy_vehicle_with_sensors():
    world = World()
    world.scene.add_default_ground_plane()

    car_path = "/World/FSAE_Car"
    chassis_path = f"{car_path}/chassis"

    #cube for now, can use car asset later 
    create_prim(
        prim_path=chassis_path,
        prim_type="Cube",
        translation=Gf.Vec3d(0, 0, 0.25),
        scale=Gf.Vec3d(1.5, 0.7, 0.5),
        semantic_label="car"
    )

    #physics for now 
    stage = world.stage
    prim = stage.GetPrimAtPath(chassis_path)

    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.CollisionAPI.Apply(prim)
    PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(500.0)

    #Adding sensors - 2 cameras and 1 LiDAR
    def add_sensor(name, prim_type, position):
        full_path = f"{car_path}/{name}"

        create_prim(
            prim_path=full_path,
            prim_type=prim_type,
            translation=Gf.Vec3d(*position)
        )

    
        sensor_prim = stage.GetPrimAtPath(full_path)
        xform = UsdGeom.Xformable(sensor_prim)
        xform.AddRotateXYZOp().Set(Gf.Vec3f(0, 0, 0))

    # Attach 1 LiDAR and 2 stereo cameras
    #These positions can be easily changed - either here or the UI
    add_sensor("lidar_front", "sensor.syntheticLidarSensor", (0.9, 0.0, 0.8))
    add_sensor("camera_left", "Camera", (-0.3, 0.15, 1.1))
    add_sensor("camera_right", "Camera", (-0.3, -0.25, 1.1))


    world.reset()
    stage.GetRootLayer().Export("assets/envs/dummy_vehicle_stage.usd")
    print(" Dummy vehicle with physics + sensors saved!")

setup_dummy_vehicle_with_sensors()
simulation_app.close()
