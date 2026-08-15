import argparse
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyrealsense2 as rs
from multiprocessing import Process, set_start_method

from arx_wrapper import ArxConfig


# Hard-bind each physical camera (by USB serial) to a logical mount position.
# Topic names downstream depend on this — never let USB enumeration order pick
# which camera is "top": with 3 RealSense devices the order flips between boots
# and any unlabeled mp4 ends up unusable for LeRobot conversion.
SERIAL_TO_NAME = dict(ArxConfig.from_env().camera_bindings)


def camera_worker(device_serial, device_name, cam_name, freq):
    """Publish color + depth for one camera on /<cam_name>_image and /<cam_name>_depth."""
    rospy.init_node(f'camera_node_{cam_name}', anonymous=True)
    image_pub = rospy.Publisher(f'/{cam_name}_image', Image, queue_size=10)
    depth_pub = rospy.Publisher(f'/{cam_name}_depth', Image, queue_size=10)
    bridge = CvBridge()
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(device_serial)
    if 'L515' in device_name:
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    elif 'D405' in device_name or 'D435' in device_name:
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    pipeline.start(config)
    align_to = rs.stream.color
    align = rs.align(align_to)
    rate = rospy.Rate(freq)
    print(f'Start to publish {cam_name} (serial={device_serial}, {device_name})')
    try:
        while not rospy.is_shutdown():
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            current_time = rospy.Time.now()

            if not color_frame:
                continue
            depth_image = np.asanyarray(aligned_depth_frame.get_data()).astype(np.float32)
            color_image = np.asanyarray(color_frame.get_data())

            color_msg = bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
            color_msg.header.stamp = current_time
            color_msg.header.frame_id = f"{cam_name}_frame"

            depth_msg = bridge.cv2_to_imgmsg(depth_image, encoding="32FC1")
            depth_msg.header.stamp = current_time
            depth_msg.header.frame_id = f"{cam_name}_frame"

            image_pub.publish(color_msg)
            depth_pub.publish(depth_msg)
            rate.sleep()
    finally:
        pipeline.stop()
        print(f"Pipeline for {cam_name} ({device_serial}) stopped.")


class RealsenseMulti:
    def __init__(self, freq=30):
        self.freq = freq
        self.device_info = []
        unknown = []
        for device in rs.context().devices:
            device_name = device.get_info(rs.camera_info.name)
            if device_name.lower() == 'platform camera':
                continue
            serial = device.get_info(rs.camera_info.serial_number)
            cam_name = SERIAL_TO_NAME.get(serial)
            if cam_name is None:
                unknown.append((serial, device_name))
                continue
            self.device_info.append((serial, device_name, cam_name))

        if unknown:
            print('WARNING: unknown camera serial(s) — skipped:')
            for s, n in unknown:
                print(f'  serial={s} ({n})')
            print('Set ARX_CAM_*_SERIAL in config/arx.local.env to use them.')

        if not self.device_info:
            raise RuntimeError(
                'No known RealSense cameras detected. '
                f'Expected serials: {list(SERIAL_TO_NAME.keys())}'
            )

        print('Camera binding:')
        for serial, name, cam in self.device_info:
            print(f'  {cam:18s} <- serial={serial} ({name})')

    def run(self):
        procs = []
        for serial, name, cam_name in self.device_info:
            p = Process(target=camera_worker, args=(serial, name, cam_name, self.freq))
            p.start()
            procs.append(p)
        try:
            for p in procs:
                p.join()
        except KeyboardInterrupt:
            for p in procs:
                p.terminate()
            print("All camera processes terminated.")


if __name__ == '__main__':
    set_start_method("spawn", force=True)
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--freq',
        type=int,
        default=30,
        help='Publishing frequency (Hz)',
    )
    args = parser.parse_args()
    realsense = RealsenseMulti(args.freq)
    realsense.run()
