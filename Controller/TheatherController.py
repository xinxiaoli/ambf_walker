

class TheatherController(object):


    def __init__(self, model):

        self._model = model

    def set_force(self, x, y, z):
        self._model.handle.set_force(x, y, z)