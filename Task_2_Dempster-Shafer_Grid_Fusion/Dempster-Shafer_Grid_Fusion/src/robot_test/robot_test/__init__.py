__all__ = ['Testsuite', 'TestConfigurationNode']

import importlib
import json
import sys

from pprint import pprint

class Testcase:
    def __init__(self, filename, testsuite_id, robot, testcase_id):
        self._action = []
        self._verifier = []
        self._robot = robot

        # open data file
        with open(filename, "r") as read_file:
            data = json.load(read_file)

        ts = list(filter(lambda ts: ts["id"] == testsuite_id, data))
        if len(ts) == 0:
            raise FileNotFoundError(f"testsuite {testsuite_id} not found")
        # found our test suite
        ts = ts[0]

        tc = list(filter(lambda tc: tc["id"] == testcase_id, ts["testcase"]))
        if len(tc) == 0:
            raise FileNotFoundError(f"testcase {testcase_id} not found")
        # found our test case
        tc = tc[0]
        
        self._id = f'{ts["id"]}.{tc["id"]}'
        self._description = f'{ts["description"]}\n{tc["description"]}'
        
        # merge test data
        action_ids = list(set(x["id"] for x in ts["action"]+tc["action"]))
        verifier_ids = list(set(x["id"] for x in ts["verifier"]+tc["verifier"]))

        # merge actions and create objects
        for id in action_ids:
            ts_action = list(filter(lambda tc: tc["id"] == id, ts["action"]))
            tc_action = list(filter(lambda tc: tc["id"] == id, tc["action"]))

            ts_params = {} if len(ts_action) == 0 else ts_action[0]["params"]
            tc_params = {} if len(tc_action) == 0 else tc_action[0]["params"]

            params = ts_params | tc_params
            module_name = tc_action[0]["module"] if len(tc_action) > 0 else ts_action[0]["module"]            

            module = importlib.import_module(".".join(module_name.split('.')[:-1]))
            self._action.append(eval(f"module.{module_name.split('.')[-1]}(id=id, robot=self._robot, params=params)"))

        # merge verifier and create objects
        for id in verifier_ids:
            ts_verifier = list(filter(lambda tc: tc["id"] == id, ts["verifier"]))
            tc_verifier = list(filter(lambda tc: tc["id"] == id, tc["verifier"]))

            ts_params = {} if len(ts_verifier) == 0 else ts_verifier[0]["params"]
            tc_params = {} if len(tc_verifier) == 0 else tc_verifier[0]["params"]

            params = ts_params | tc_params
            module_name = tc_verifier[0]["module"] if len(tc_verifier) > 0 else ts_verifier[0]["module"]            

            module = importlib.import_module(".".join(module_name.split('.')[:-1]))
            self._verifier.append(eval(f"module.{module_name.split('.')[-1]}(id=id, robot=self._robot, params=params)"))

    def _shutdown(self, error: Exception):
        print(f"error: {error}")
        for a in self._action:
            a.shutdown()
        for v in self._verifier:
            print(f"{v.getID()}: {v.getLastResult()}")
        self._robot.simulationQuit(99)

    def execute(self):
        exit = False
        error = None
        for v in self._verifier:
            try:
                v.execute()
            except Exception as e:
                exit = True
                error = e
                break
        if exit:
            self._shutdown(error)
            return

        exit = False
        for a in self._action:
            try:
                a.execute()
            except Exception as e:
                exit = True
                error = e
                break
        if exit:
            self._shutdown(error)
            return

