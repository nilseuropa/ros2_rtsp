import os
import threading
import time
import subprocess
import shutil

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import UInt8MultiArray


class RtspCameraWorker:
    def __init__(
        self,
        node,
        index,
        ip,
        username,
        password,
        rtsp_path,
        low_latency,
        drop_frames,
        max_grab,
    ):
        self.node = node
        self.index = index
        self.ip = ip
        self.username = username
        self.password = password
        self.rtsp_path = rtsp_path
        self.low_latency = low_latency
        self.drop_frames = drop_frames
        self.max_grab = max_grab
        self.stop_event = threading.Event()

        base_ns = f"/camnet/cam_{index}"
        self.image_pub = node.create_publisher(Image, f"{base_ns}/image_raw", 10)
        self.info_pub = node.create_publisher(CameraInfo, f"{base_ns}/camera_info", 10)
        self.audio_pub = node.create_publisher(UInt8MultiArray, f"{base_ns}/audio", 10)

        self.video_thread = threading.Thread(target=self._video_loop, daemon=True)
        self.audio_thread = threading.Thread(target=self._audio_loop, daemon=True)

    def start(self):
        self.video_thread.start()
        self.audio_thread.start()

    def stop(self):
        self.stop_event.set()

    def _rtsp_url(self):
        path = self.rtsp_path.lstrip("/")
        return f"rtsp://{self.username}:{self.password}@{self.ip}:554/{path}"

    def _video_loop(self):
        url = self._rtsp_url()
        backoff = 1.0
        while not self.stop_event.is_set() and rclpy.ok():
            if self.low_latency:
                os.environ.setdefault(
                    "OPENCV_FFMPEG_CAPTURE_OPTIONS",
                    "rtsp_transport;tcp|fflags;nobuffer|flags;low_delay|max_delay;0|buffer_size;0",
                )
            cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                self.node.get_logger().info(f"cam_{self.index}: RTSP video connected")
                backoff = 1.0
                while not self.stop_event.is_set() and rclpy.ok():
                    if self.drop_frames:
                        ok = cap.grab()
                        if ok:
                            for _ in range(self.max_grab):
                                if not cap.grab():
                                    break
                            ok, frame = cap.retrieve()
                        else:
                            frame = None
                    else:
                        ok, frame = cap.read()
                    if not ok or frame is None:
                        self.node.get_logger().warn(
                            f"cam_{self.index} ({self.ip}): video frame failed, reconnecting"
                        )
                        break
                    self._publish_frame(frame)
                cap.release()
            else:
                cap.release()
                self.node.get_logger().warn(
                    f"cam_{self.index} ({self.ip}): RTSP video open failed, retrying"
                )
            time.sleep(backoff)
            backoff = min(backoff * 2.0, 10.0)

    def _publish_frame(self, frame):
        height, width = frame.shape[:2]
        msg = Image()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = f"cam_{self.index}_optical"
        msg.height = height
        msg.width = width
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = width * 3
        msg.data = frame.tobytes()
        self.image_pub.publish(msg)

        info = CameraInfo()
        info.header = msg.header
        info.height = height
        info.width = width
        self.info_pub.publish(info)

    def _audio_loop(self):
        if shutil.which("ffmpeg") is None:
            self.node.get_logger().warn(
                f"cam_{self.index}: ffmpeg not found, audio disabled"
            )
            return

        url = self._rtsp_url()
        cmd = [
            "ffmpeg",
            "-rtsp_transport",
            "tcp",
            "-i",
            url,
            "-vn",
            "-acodec",
            "pcm_s16le",
            "-ac",
            "1",
            "-ar",
            "16000",
            "-f",
            "s16le",
            "-",
        ]

        backoff = 1.0
        while not self.stop_event.is_set() and rclpy.ok():
            try:
                proc = subprocess.Popen(
                    cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL
                )
            except Exception as exc:
                self.node.get_logger().warn(
                    f"cam_{self.index}: audio start failed: {exc}"
                )
                time.sleep(backoff)
                backoff = min(backoff * 2.0, 10.0)
                continue

            self.node.get_logger().info(
                f"cam_{self.index}: RTSP audio connected (16kHz mono s16le)"
            )
            backoff = 1.0
            if proc.stdout is None:
                proc.terminate()
                time.sleep(backoff)
                continue

            chunk_size = 16000 * 2 // 10  # 100ms of audio
            while not self.stop_event.is_set() and rclpy.ok():
                data = proc.stdout.read(chunk_size)
                if not data:
                    self.node.get_logger().warn(
                        f"cam_{self.index}: audio stream ended, reconnecting"
                    )
                    break
                msg = UInt8MultiArray()
                msg.data = list(data)
                self.audio_pub.publish(msg)

            proc.terminate()
            try:
                proc.wait(timeout=1.0)
            except Exception:
                proc.kill()
            time.sleep(backoff)
            backoff = min(backoff * 2.0, 10.0)


class RtspMultiNode(Node):
    def __init__(self):
        super().__init__("rtsp_multi")
        self.declare_parameter(
            "ip_addresses",
            Parameter.Type.STRING_ARRAY,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING_ARRAY
            ),
        )
        self.declare_parameter(
            "user_name",
            "",
            descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING),
        )
        self.declare_parameter(
            "user_password",
            "",
            descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING),
        )
        self.declare_parameter("rtsp_path", "stream2")
        self.declare_parameter("low_latency", True)
        self.declare_parameter("drop_frames", True)
        self.declare_parameter("max_grab", 5)

        self._workers = []
        self._start_workers()

    def _start_workers(self):
        ip_addresses = list(self.get_parameter("ip_addresses").value)
        username = self.get_parameter("user_name").value
        password = self.get_parameter("user_password").value
        rtsp_path = self.get_parameter("rtsp_path").value
        low_latency = bool(self.get_parameter("low_latency").value)
        drop_frames = bool(self.get_parameter("drop_frames").value)
        max_grab = int(self.get_parameter("max_grab").value)

        if not ip_addresses:
            self.get_logger().warn("No ip_addresses provided")
            return
        if not username or not password:
            self.get_logger().warn("user_name or user_password missing")

        for index, ip in enumerate(ip_addresses):
            worker = RtspCameraWorker(
                self,
                index,
                ip,
                username,
                password,
                rtsp_path,
                low_latency,
                drop_frames,
                max_grab,
            )
            self._workers.append(worker)
            worker.start()

    def destroy_node(self):
        for worker in self._workers:
            worker.stop()
        super().destroy_node()


def main():
    rclpy.init()
    node = RtspMultiNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
