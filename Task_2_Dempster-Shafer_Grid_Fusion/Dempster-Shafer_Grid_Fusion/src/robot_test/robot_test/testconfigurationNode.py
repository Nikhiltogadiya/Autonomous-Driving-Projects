import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters

class TestConfigurationNode(Node):

    def __init__(self):
        super().__init__('TestConfiguration')

        self.declare_parameter("servername", "")
        self.declare_parameter("testdefinition", "")
        self.declare_parameter("testsuite", "")
        self.declare_parameter("testcase", "")
        self.__servername = self.get_parameter(
            'servername').get_parameter_value().string_value
        self.__testdefinition = self.get_parameter(
            'testdefinition').get_parameter_value().string_value
        self.__testsuite = self.get_parameter(
            'testsuite').get_parameter_value().string_value
        self.__testcase = self.get_parameter(
            'testcase').get_parameter_value().string_value
        
        if self.__testdefinition =="" or self.__testsuite =="" or self.__testcase =="":
            self.get_logger().info('test parameter not set')
        else:
            self.cli = self.create_client(SetParameters, '/' + self.__servername + '/set_parameters')
            while not self.cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')
            self.req = SetParameters.Request()

            self.send_request("testdefinition", self.__testdefinition)
            self.send_request("testsuite",self.__testsuite)
            self.send_request("testcase",self.__testcase)
            self.get_logger().info('test parameter updates sent')


    def send_request(self, param_name, param_value):
        if isinstance(param_value, float):
            val = ParameterValue(double_value=param_value, type=ParameterType.PARAMETER_DOUBLE)
        elif isinstance(param_value, int):
            val = ParameterValue(integer_value=param_value, type=ParameterType.PARAMETER_INTEGER)
        elif isinstance(param_value, str):
            val = ParameterValue(string_value=param_value, type=ParameterType.PARAMETER_STRING)
        elif isinstance(param_value, bool):
            val = ParameterValue(bool_value=param_value, type=ParameterType.PARAMETER_BOOL)
        self.req.parameters = [Parameter(name=param_name, value=val)]
        self.future = self.cli.call_async(self.req)
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.future.done():
                try:
                    response = self.future.result()
                    if response[0].successful:
                        return True
                except Exception as e:
                    pass
                return False
        
def main(args=None):
    rclpy.init(args=args)
    node = TestConfigurationNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
