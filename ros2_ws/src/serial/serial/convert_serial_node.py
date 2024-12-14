class ConvertSerialNode(Node):

    def __init__(self):
        super().__init__('convert_serial_node')

        # Declare parameters
        self.declare_parameter('motor_control_topic', 'motor_control')
        self.declare_parameter('actuator_target_topic', 'actuator_target')
        self.declare_parameter('status_leds_topic', 'status_leds')
        self.declare_parameter('lcd_input_topic', 'lcd_input')
        self.declare_parameter('actuator_feedback_topic', 'actuator_feedback')
        self.declare_parameter('serial_read_topic', '/serial_read')
        self.declare_parameter('serial_write_topic', '/serial_write')

        # Get parameters
        motor_control_topic = self.get_parameter('motor_control_topic').get_parameter_value().string_value
        actuator_target_topic = self.get_parameter('actuator_target_topic').get_parameter_value().string_value
        status_leds_topic = self.get_parameter('status_leds_topic').get_parameter_value().string_value
        lcd_input_topic = self.get_parameter('lcd_input_topic').get_parameter_value().string_value
        actuator_feedback_topic = self.get_parameter('actuator_feedback_topic').get_parameter_value().string_value
        serial_read_topic = self.get_parameter('serial_read_topic').get_parameter_value().string_value
        serial_write_topic = self.get_parameter('serial_write_topic').get_parameter_value().string_value

        #TODO: fix message types
        #TODO: fix callback functions
        #TODO: Convert to serial message3

        # Create subscribers
        self.motor_control_subscriber = self.create_subscription(
            String,
            motor_control_topic,
            self.motor_control_callback,
            10)
        
        self.actuator_target_subscriber = self.create_subscription(
            String,
            actuator_target_topic,
            self.actuator_target_callback,
            10)
        
        self.status_leds_subscriber = self.create_subscription(
            String,
            status_leds_topic,
            self.status_leds_callback,
            10)
        
        self.lcd_input_subscriber = self.create_subscription(
            String,
            lcd_input_topic,
            self.lcd_input_callback,
            10)
        
        self.serial_read_subscriber = self.create_subscription(
            String,
            serial_read_topic,
            self.serial_read_callback,
            10)

        # Create publishers
        self.actuator_feedback_publisher = self.create_publisher(
            String,
            actuator_feedback_topic,
            10)
        
        self.serial_write_publisher = self.create_publisher(
            String,
            serial_write_topic,
            10)

    def motor_control_callback(self, msg):
        self.get_logger().info(f'Received motor control: {msg.data}')

    def actuator_target_callback(self, msg):
        self.get_logger().info(f'Received actuator target: {msg.data}')

    def status_leds_callback(self, msg):
        self.get_logger().info(f'Received status leds: {msg.data}')

    def lcd_input_callback(self, msg):
        self.get_logger().info(f'Received lcd input: {msg.data}')

    def serial_read_callback(self, msg):
        self.get_logger().info(f'Received serial read: {msg.data}')
        # Process the serial read data and publish to serial_write
        self.serial_write_publisher.publish(msg)