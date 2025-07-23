print(">>>> MINIMAL V3 fusion_driver.py: Module is being parsed/imported by Python.", flush=True)

try:
    import rclpy
    print(">>>> MINIMAL V3 fusion_driver.py: Successfully imported rclpy.", flush=True)
except ImportError as e_rclpy:
    print(f">>>> MINIMAL V3 fusion_driver.py: FAILED to import rclpy: {e_rclpy}", flush=True)
    rclpy = None
except Exception as e_generic_rclpy:
    print(f">>>> MINIMAL V3 fusion_driver.py: UNEXPECTED ERROR importing rclpy: {e_generic_rclpy}", flush=True)
    rclpy = None

class FusionDriver:
    print(">>>> MINIMAL V3 fusion_driver.py: FusionDriver class is being defined.", flush=True)

    def __init__(self, webots_node_argument, properties_argument):
        # THIS IS THE VERY FIRST LINE OF __init__
        print(">>>> MINIMAL V3 FusionDriver __init__: Entered constructor SUCCESSFULLY.", flush=True)
        
        # Just print the types of the arguments to see what C++ is passing
        try:
            print(f">>>> MINIMAL V3 FusionDriver __init__: webots_node_argument type: {type(webots_node_argument)}", flush=True)
            print(f">>>> MINIMAL V3 FusionDriver __init__: properties_argument type: {type(properties_argument)}", flush=True)
        except Exception as e_print_types:
            print(f">>>> MINIMAL V3 FusionDriver __init__: Error printing argument types: {e_print_types}", flush=True)

        # Defer using webots_node_argument until we confirm __init__ is entered
        # self.__node = webots_node_argument
        # self.__robot = self.__node.robot # This line might fail if webots_node_argument is not what's expected
        
        # Try a very safe operation with webots_node_argument if rclpy was imported
        if rclpy and webots_node_argument is not None:
            if hasattr(webots_node_argument, 'get_logger'):
                try:
                    logger = webots_node_argument.get_logger()
                    logger.info("MINIMAL V3 FusionDriver: Logger obtained from webots_node_argument.")
                    print(">>>> MINIMAL V3 FusionDriver __init__: Logger obtained and used.", flush=True)
                except Exception as e_log:
                    print(f">>>> MINIMAL V3 FusionDriver __init__: Error using logger from webots_node_argument: {e_log}", flush=True)
            else:
                print(">>>> MINIMAL V3 FusionDriver __init__: webots_node_argument does not have get_logger.", flush=True)
        elif not rclpy:
            print(">>>> MINIMAL V3 FusionDriver __init__: rclpy was not imported, cannot test logger.", flush=True)
        else: # rclpy imported but webots_node_argument is None
             print(">>>> MINIMAL V3 FusionDriver __init__: webots_node_argument is None.", flush=True)


        print(">>>> MINIMAL V3 FusionDriver __init__: Constructor finished.", flush=True)

    def step(self):
        # print(">>>> MINIMAL V3 FusionDriver step: Called.", flush=True)
        # For now, step does nothing to keep it simple
        pass

print(">>>> MINIMAL V3 fusion_driver.py: Module parsing complete. FusionDriver class defined.", flush=True)