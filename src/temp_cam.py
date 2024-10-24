import cv2
import gi
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np

gi.require_version('Gst', '1.0')
from gi.repository import Gst

class UDPStreamPublisher:
    def __init__(self):
        rospy.init_node('udp_stream_publisher', anonymous=True)
        self.image_pub = rospy.Publisher('/camera/image_rect', Image, queue_size=10)
        self.camera_info_pub = rospy.Publisher('/camera/camera_info', CameraInfo, queue_size=10)
        self.bridge = CvBridge()

        # Initialize GStreamer
        Gst.init(None)

        # Build the GStreamer pipeline
        pipeline_str = (
            'udpsrc port=5602 ! '
            'application/x-rtp, encoding-name=H264, payload=96 ! '
            'rtpjitterbuffer ! '
            'rtph264depay ! '
            'avdec_h264 ! '
            'videoconvert ! '
            'video/x-raw,format=BGR ! '
            'appsink emit-signals=true'
        )

        # Create the pipeline
        self.pipeline = Gst.parse_launch(pipeline_str)
        self.appsink = self.pipeline.get_by_name('appsink0')
        self.appsink.connect('new-sample', self.new_sample)

        # Set camera info parameters
        self.camera_info = self.get_camera_info()

        # Start the pipeline
        self.pipeline.set_state(Gst.State.PLAYING)

    def get_camera_info(self):
        # Create and populate a CameraInfo message
        camera_info = CameraInfo()
        camera_info.header.frame_id = "camera_frame"
        camera_info.width = 640  # Replace with actual width
        camera_info.height = 480  # Replace with actual height
        camera_info.distortion_model = "plumb_bob"
        camera_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]  # Update with actual distortion coefficients
        camera_info.K = [535.4, 0.0, 320.1, 0.0, 539.2, 247.6, 0.0, 0.0, 1.0]  # Camera matrix
        camera_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]  # Rectification matrix
        camera_info.P = [535.4, 0.0, 320.1, 0.0, 0.0, 539.2, 247.6, 0.0, 0.0, 0.0, 1.0, 0.0]  # Projection matrix
        camera_info.binning_x = 1
        camera_info.binning_y = 1
        camera_info.roi.x_offset = 0
        camera_info.roi.y_offset = 0
        camera_info.roi.height = 0
        camera_info.roi.width = 0
        camera_info.roi.do_rectify = False

        return camera_info

    def new_sample(self, sink):
        sample = sink.emit('pull-sample')
        if sample:
            buf = sample.get_buffer()
            caps = sample.get_caps()
            success, info = buf.map(Gst.MapFlags.READ)

            if not success:
                rospy.logerr("Failed to map buffer.")
                return Gst.FlowReturn.ERROR

            # Extract frame dimensions from caps
            width = caps.get_structure(0).get_value('width')
            height = caps.get_structure(0).get_value('height')

            # Check if the buffer size matches the expected size
            data_size = len(info.data)
            expected_size = width * height * 3  # For BGR format

            if data_size != expected_size:
                rospy.logerr(f"Unexpected buffer size: {data_size}. Skipping frame.")
                buf.unmap(info)
                return Gst.FlowReturn.OK

            # Convert buffer to a NumPy array and reshape to image
            frame = np.frombuffer(info.data, dtype=np.uint8).reshape((height, width, 3))

            # Publish the frame to /camera/image_rect topic
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            current_time = rospy.Time.now()
            image_msg.header.stamp = current_time

            self.image_pub.publish(image_msg)

            # Publish CameraInfo with the same timestamp
            self.camera_info.header.stamp = current_time
            self.camera_info_pub.publish(self.camera_info)

            buf.unmap(info)
        return Gst.FlowReturn.OK

    def shutdown(self):
        self.pipeline.set_state(Gst.State.NULL)

if __name__ == '__main__':
    try:
        node = UDPStreamPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        node.shutdown()
