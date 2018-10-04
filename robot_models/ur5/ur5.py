from models import ColladaMesh
from pythreejs import Object3D, AxesHelper
import numpy as np
from transformations import quaternion_from_matrix


class Link(Object3D):
    def __init__(self):
        Object3D.__init__(self)

    def __call__(self, trf):
        self.quaternion = quaternion_from_matrix(trf).tolist()
        self.position = (trf[:3, 3]).tolist()


class BaseLink(Link):
    def __init__(self):
        Object3D.__init__(self)
        mesh = ColladaMesh('ur5/visual/base.dae')
        mesh.rotateZ(np.pi)
        self.add(mesh)
        axes = AxesHelper(size=0.5)
        self.add(axes)


class Link1(Link):
    def __init__(self):
        Object3D.__init__(self)
        mesh = ColladaMesh('ur5/visual/shoulder.dae')
        mesh.rotateX(-np.pi/2)
        mesh.rotateZ(np.pi)
        self.mesh = mesh
        self.add(mesh)
        axes = AxesHelper(size=0.5)
        self.add(axes)


class Link2(Link):
    def __init__(self):
        Object3D.__init__(self)
        mesh = ColladaMesh('ur5/visual/upperarm.dae')
        mesh.rotateX(np.pi/2)
        mesh.rotateY(-np.pi/2)
        mesh.position = (0.425, 0, 0.13585)
        self.mesh = mesh
        self.add(mesh)
        axes = AxesHelper(size=0.5)
        self.add(axes)


class Link3(Link):
    def __init__(self):
        Object3D.__init__(self)
        mesh = ColladaMesh('ur5/visual/forearm.dae')
        self.mesh = mesh
        mesh.rotateX(np.pi/2)
        mesh.position = (0.39225, 0., -0.1197+0.13585)
        mesh.rotateY(-np.pi/2)
        self.add(mesh)
        axes = AxesHelper(size=0.5)
        self.add(axes)


class Link4(Link):
    def __init__(self):
        Object3D.__init__(self)
        mesh = ColladaMesh('ur5/visual/wrist1.dae')
        mesh.position = (0.0, -0.10915 + (-0.1197+0.13585), 0)
        self.mesh = mesh
        self.add(mesh)
        axes = AxesHelper(size=0.5)
        self.add(axes)


class Link5(Link):
    def __init__(self):
        Object3D.__init__(self)
        mesh = ColladaMesh('ur5/visual/wrist2.dae')
        mesh.position = (0.0, 0.09465, 0)
        mesh.rotateX(np.pi/2)
        self.mesh = mesh
        self.add(mesh)
        axes = AxesHelper(size=0.5)
        self.add(axes)


class Link6(Link):
    def __init__(self):
        Object3D.__init__(self)
        mesh = ColladaMesh('ur5/visual/wrist3.dae')
        mesh.position = (0.0, 0.0, -0.0823)
        mesh.rotateX(np.pi/2)
        self.mesh = mesh
        self.add(mesh)
        axes = AxesHelper(size=0.5)
        self.add(axes)
