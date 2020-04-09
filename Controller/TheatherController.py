
class TheatherController(object):

    def __init__(self, model):

        self._model = model

    def set_force(self, z=35):
        self._model.handle.set_force(0, 0, z)

    def set_pos(self, roll, height):
        self._model.handle.set_rpy(roll, 0, 0)
        self._model.handle.set_pos(0, 0, height)
