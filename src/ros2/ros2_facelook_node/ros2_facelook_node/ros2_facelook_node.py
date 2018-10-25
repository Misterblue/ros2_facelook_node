# Copyright 2018 Robert Adams
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import io
import queue
import threading
import time
import sys

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import MultiArrayDimension

from ros2_adafruit_pwmhat_msgs.msg import PWMPinAngle, PWMAngle

class ROS2_facelook_node(Node):

    def __init__(self):
        super().__init__('ros2_facelook_node', namespace='raspicam')

        self.param_image_bbox_topic = str(self.get_parameter_or('bounding_box_topic', '/raspicam/found_faces'))
        self.param_pwm_topic = str(self.get_parameter_or('pwm_topic', '/pwmhatter/angle'))
        self.param_angle_step = float(self.get_parameter_or('angle_step', 5.0))
        self.param_max_angle = float(self.get_parameter_or('max_angle', 80.0))

        self.initialize_pwm_publisher()
        self.initialize_processing_queue()
        self.initialize_bounding_box_subscriber()

    def destroy_node(self):
        # overlay Node function called when class is being stopped and camera needs closing
        super().destroy_node()

    def initialize_bounding_box_subscriber(self):
        # Setup subscription for incoming bounding box info
        self.receiver = self.create_subscription(
                        Int32MultiArray, self.param_bbox_input_topic, self.receive_bounding_box)

    def initialize_processing_queue(self):
        # Create a queue and a thread that processes messages in the queue
        self.queue_lock = threading.Lock()

        self.bbox_queue = queue.Queue()
        # self.bbox_queue = queue.SimpleQueue()  # introduced in Python 3.7

        # thread to read images placed in the queue and process them
        self.processor_event = threading.Event()
        self.processor = threading.Thread(target=self.process_bounding_boxes, name='bounding box')

        self.processor.start()

    def initialize_pwm_publisher():
        self.pwmmer = PWMmer(self, self.param_pwm_topic, self.param_max_angle, -self.param_max_angle, self.get_logger())

    def stop_workers(self):
        # if workers are initialized and running, tell them to stop and wait until stopped
        if hasattr(self, 'processor_event') and self.processor_event != None:
            self.processor_event.set()
        if hasattr(self, 'processor') and self.processor.is_alive():
            self.processor.join()

    def receive_bounding_box(self, msg):
        if type(msg) != type(None) and hasattr(msg, 'data'):
            self.get_logger().debug(F"FLooker: receive_bbox. dataLen={len(msg.data)}")
            self.bbox_queue.put(msg)

    def process_bounding_boxes(self):
        # Take bounding boxes from the queue and send angle commands to the camera

        # Initialize camera position
        self.send_pwm_commands(0, 0)

        # Loop for each bounding box info and update the camera movement
        while True:
            if self.processor_event.is_set():
                break
            try:
                msg = self.bbox_queue.get(block=True, timeout=2)
            except queue.Empty:
                msg = None
            if self.processor_event.is_set():
                break
            if type(msg) != type(None):
                bboxes = AccessInt32MultiArray(msg)
                self.get_logger().debug('FLooker: process_bounding_boxes. Boxes=%s' % (bboxes.rows) )
                width = bboxes.get(0,3)
                widthhalf = width / 2
                height = bboxes.get(0, 4)
                heighthalf = height / 2

                # loop over all bounding boxes and computer the average center
                wcenter = 0
                hcenter = 0
                for ii in range(1, bboxes.rows - 1):
                    wcenter = wcenter + (boxes.get(ii, 2) - bboxes.get(ii, 0))
                    hcenter = hcenter + (boxes.get(ii, 3) - bboxes.get(ii, 1))
                waverage = wcenter / ( bboxes.rows - 1)
                haverage = hcenter / ( bboxes.rows - 1)

                # positive deltas mean above the middle and negative deltas mean below the middle
                wdelta = (width / 2) - waverage
                hdelta = (height / 2) - haverage

                if wdelta <= -widthhalf or wdelta >= widthhalf or hdelta <= -heighthalf or hdelta >= heighthalf:
                    self.get_logger().error('FLooker: deltas wrong! dim=%s/%s, avg=%s/%s, delta=%s/%s'
                                % ( width, height, waverage, haverage, wdelta, hdelta) )
                else:
                    target_pan_angle = self.pan_angle + (self.param_angle_step * sign(hdelta))
                    target_tilt_angle = self.tilt_angle + (self.param_angle_step * sign(wdelta))
                    self.send_pwm_commands(target_pan_angle, target_tilt_angle)

    def send_pwm_commands(target_pan_angle, target_tilt_angle):
        # Send command to PWM channels if the desired angle has changed
        self.pwmmer('pan', target_pan_angle)
        self.pwmmer('tilt', target_tilt_angle)

        if target_pan_angle != self.pan_angle:
            if self.pwmmer('pan', target_pan_angle):
                self.pan_angle = target_pan_angle
                self.get_logger().debug('FLooker: sending chan=%s, angle=%s' % ('pan', target_pan_angle))
            else:
                self.get_logger().error('FLooker: target pan angle failed! targets=%s/%s'
                                % (target_pan_angle, target_tilt_angle) )
        if target_tilt_angle != self.tilt_angle:
            if self.pwmmer('tilt', target_tilt_angle):
                self.tilt_angle = target_tilt_angle
                self.get_logger().debug('FLooker: sending chan=%s, angle=%s' % ('tilt', target_tilt_angle))
            else:
                self.get_logger().error('FLooker: target tilt angle failed! targets=%s/%s'
                                % (target_pan_angle, target_tilt_angle) )
                    
    def get_parameter_or(self, param, default):
        # Helper function to return value of a parameter or a default if not set
        ret = None
        param_desc = self.get_parameter(param)
        if param_desc.type_== Parameter.Type.NOT_SET:
            ret = default
        else:
            ret = param_desc.value
        return ret

class PWMmer:
    # Small class to hold current state of PWM channel
    def __init__(self, node, topic, minVal, maxVal, logger=None):
        self.publisher = publisher
        self.minVal = minVal
        self.maxVal = maxVal
        self.logger = logger
        self.channels = {}

        self.publisher = node.create_publisher(PWMArray, topic)

    def setPWM(self, channel, angle):
        # Send the message to set the given PWM channel
        if not channel in self.channels:
            self.channels[channel] = minVal - 1000

        if angle != self.channels[channel]:
            if angle >= self.maxVal or angle <= minVal:
                self.logger.error('FLooker: target pan angle failed! targets=%s/%s'
                                % (target_pan_angle, target_tilt_angle) )
            else:
                msg = PWMAngle()
                msg.chan = channel
                msg.angle = angle
                msg.angle_units = PWMAngle.DEGREES
                self.publisher.publish(msg)
                self.channel[channel] = angle


class AccessInt32MultiArray:
    # Wrap a multi-access array with functions for 2D access
    def __init__(self, arr):
        self.arr = arr
        self.arr_width = int(arr.layout.dim['width'].size)
        self.arr_height = int(arr.layout.dim['height'].size)

    def rows(self):
        # return the number of rows in the multi-array
        return self.arr_height

    def get(self, ww, hh):
        # return the entry at column 'ww' and row 'hh'
        return data[ww + ( hh * self.arr_width)]
        

class CodeTimer:
    # A little helper class for timing blocks of code
    def __init__(self, logger, name=None):
        self.logger = logger
        self.name = " '"  + name + "'" if name else ''

    def __enter__(self):
        self.start = time.clock()

    def __exit__(self, exc_type, exc_value, traceback):
        self.took = (time.clock() - self.start) * 1000.0
        self.logger('Code block' + self.name + ' took: ' + str(self.took) + ' ms')

def main(args=None):
    rclpy.init(args=args)

    ffNode = ROS2_facelook_node()

    try:
        rclpy.spin(ffNode)
    except KeyboardInterrupt:
        ffNode.get_logger().info('FLooker: Keyboard interrupt')

    ffNode.stop_workers()

    ffNode.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
