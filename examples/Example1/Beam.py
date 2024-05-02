import sys

sys.path.append("/workspace/build/dev-release/lib")

import PySeaFood
import numpy as np
import polyscope as ps
import polyscope.imgui as psim

import meshio

mesh = meshio.read("box.obj")
cube = PySeaFood.Mesh()
cube.points = mesh.points
cube.faces = mesh.get_cells_type("triangle")

mesh = meshio.read("maze.obj")
maze = PySeaFood.Mesh()
maze.points = mesh.points
maze.faces = mesh.get_cells_type("triangle")

ps.init()
ps.register_surface_mesh("maze", mesh.points, mesh.get_cells_type("triangle"))

x0 = np.array([0.5, 0.5, 0.5]) + np.array([0.1,0.1,0.1]) * np.random.rand(128, 3)
x1 = np.array([0.2, 0.8, 0.5]) + np.array([0.1,0.1,0.1]) * np.random.rand(128, 3)
x2 = np.array([-0.5, 0.5, 0.5]) + np.array([0.1,0.1,0.1]) * np.random.rand(128, 3)
boid = PySeaFood.Boid()
boid.x = np.vstack([x0, x1, x2])
boid.v = 0.01 * np.random.rand(boid.x.shape[0], boid.x.shape[1])

cloud = ps.register_point_cloud("Boid", boid.x)
cloud.add_vector_quantity("v", boid.x, enabled=True)

scene = PySeaFood.Scene()
scene.boid = boid
scene.meshes = [cube, maze]
scene.commit()

simulation = PySeaFood.Simulation()
simulation.cohesionVelocityScale = 0.2
simulation.crowdedVelocityScale = 0.05
simulation.steeringScale = 0.1
simulation.avoidObstacleScale = 100

simulation.perceptionRadius = 0.4
simulation.crowdedRadius = 0.05
simulation.maxVelocity = 0.3

import time
h = 0
t = 0
def simulate():
    global h
    global t

    changes, simulation.cohesionVelocityScale = psim.SliderFloat("Cohesion scale", simulation.cohesionVelocityScale, v_min = 0, v_max = 1)
    changes, simulation.crowdedVelocityScale = psim.SliderFloat("Crowded scale", simulation.crowdedVelocityScale, v_min = 0, v_max = 1)
    changes, simulation.steeringScale = psim.SliderFloat("Steering scale", simulation.steeringScale, v_min = 0, v_max = 1)
    changes, simulation.avoidObstacleScale = psim.SliderFloat("Avoid obstacles scale", simulation.avoidObstacleScale, v_min = 0, v_max = 10)
    changes, simulation.perceptionRadius = psim.SliderFloat("Perception radius", simulation.perceptionRadius, v_min = 0, v_max = 1)
    changes, simulation.crowdedRadius = psim.SliderFloat("Crowded radius", simulation.crowdedRadius, v_min = 0, v_max = 1)
    changes, simulation.maxVelocity = psim.SliderFloat("Velocity", simulation.maxVelocity, v_min = 0, v_max = 1)
    changes, simulation.randomization = psim.SliderFloat("Randomization", simulation.randomization, v_min = 0, v_max = 1)

    start = time.time()
    simulation.step(scene, h)
    t += h
    cloud.update_point_positions(scene.boid.x)
    cloud.add_vector_quantity("v", scene.boid.v, enabled=True)
    h = time.time() - start

ps.set_user_callback(simulate)
ps.show()

