__all__ = ["Verifier", "Pose", "DoNotHitAnything","sqDist"]

def sqDist(vec1: list[float], vec2:list[float]) -> float:
    assert len(vec1) == len(vec2)
    return sum(
        list(map(lambda x: (vec1[x]-vec2[x])*(vec1[x]-vec2[x]), range(len(vec1))))
    )


class Verifier:
    def __init__(self, id, robot, params={}):
        self._id = id
        self._params = params
        self._robot = robot
        self._last = False

    def getID(self) -> str:
        return self._id

    def getLastResult(self) -> bool:
        return self._last
    
    def execute(self):
        pass

import sys
class Pose(Verifier):
    def __init__(self, id, robot, params={}):
        super().__init__(id=id, robot=robot, params=params)



    def execute(self):
        node = self._robot.getFromDef(self._params["def_name"])
        if sqDist(node.getPosition(), self._params["translation"]) < self._params["translation_deviation"]*self._params["translation_deviation"]:
            if sqDist(node.getOrientation(), self._params["orientation"]) < self._params["orientation_deviation"]*self._params["orientation_deviation"]:
                self._last = True


class DoNotHitAnything(Verifier):
    def __init__(self, id, robot, params={}):
        super().__init__(id=id, robot=robot, params=params)
        # start with everything good
        self._last = True

        self._sensors = list(map(lambda x: self._robot.getDevice(x), self._params["sensors"]))

        for s in self._sensors:
            s.enable(100) # 100ms sample

    def execute(self):
        if self._last:
            for s in self._sensors:
                if s.getValue() < self._params["min_distance"]:
                    self._last = False
                    raise Exception(f"robot hit an obstacle with touch sensor: {s.getName()}")

#eof