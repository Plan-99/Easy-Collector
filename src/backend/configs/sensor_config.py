SENSOR_CONFIGS = {
    'realsense_camera': {
        'serial_number': {'default': '00000000', 'type': str}
    },
    'custom_sensor': {
        'process_cmd': {'default': 'rosrun my_package my_sensor_node', 'type': str},
        'topic_name': {'default': 'custom_sensor_topic', 'type': str},
    }
}