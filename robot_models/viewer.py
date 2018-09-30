from pythreejs import (DirectionalLight, PointLight, AmbientLight,
                       PerspectiveCamera, Scene, OrbitControls, Renderer)
from IPython.display import display


class Viewer:
    def __init__(self):

        key_light = DirectionalLight(color='white', position=[
                                     3, 3, 3], intensity=0.66)
        near = 0.01
        far = 1000
        width = 768
        height = 512

        c = PerspectiveCamera(40, width/height, near, far)
        c.position = [3, 3, 3]
        c.up = [0, 0, 1]

        c.add(key_light)

        pl = PointLight(color='0xffffff', intensity=0.1, position=[3, 3, 3])

        self._scene = Scene()
        self._scene.background = "#111111"
        self._scene.add(AmbientLight())
        self._scene.add(pl)

        renderer = Renderer(camera=c,
                            scene=self._scene,
                            antialias=True,
                            controls=[OrbitControls(controlling=c)],
                            height=height, width=width)
        display(renderer)

    def add(self, obj):
        self._scene.add(obj)
