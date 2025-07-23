__all__ = ["Action", "Timeout", "Movie", "MoveNode"]

import time
import os.path

class Action:
    def __init__(self, id, robot, params={}):
        self._id = id
        self._params = params
        self._robot = robot

    
    def execute(self):
        pass

    def shutdown(self):
        pass

class Timeout(Action):
    def __init__(self, id, robot, params={}):
        super().__init__(id=id, robot=robot, params=params)

    def execute(self):
        if self._robot.getTime() > self._params["timeout"]:
            raise Exception(f"timeout after {self._params['timeout']}")

class MoveNode(Action):
    def __init__(self, id, robot, params={}):
        super().__init__(id=id, robot=robot, params=params)
        self.__moved = False

    def execute(self):
        if self._robot.getTime() > self._params["after"] and not self.__moved:
            node = self._robot.getFromDef(self._params["def_name"])
            node.getField("translation").setSFVec3f(self._params["translation"])
            node.getField("rotation").setSFRotation(self._params["rotation"])
            self.__moved = True


class Movie(Action):
    def __init__(self, id, robot, params={}):
        super().__init__(id=id, robot=robot, params=params)
        self.__moviestarted = False
        if os.path.exists(self._params["filename"]):
            os.unlink(self._params["filename"])

    def execute(self):
        if not self.__moviestarted:
            self._robot.movieStartRecording(self._params["filename"], self._params["width"], self._params["height"], "ignored-codec", 50, 1, False)
            self.__moviestarted = True

    def shutdown(self):
        if self.__moviestarted:
            self._robot.movieStopRecording()
            while not os.path.exists(self._params["filename"]):
                time.sleep(1)

#eof