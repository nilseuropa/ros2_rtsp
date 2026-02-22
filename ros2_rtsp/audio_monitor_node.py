import queue
import shutil
import subprocess
import sys
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray


class AudioMonitorNode(Node):
    def __init__(self):
        super().__init__("audio_monitor")

        self.declare_parameter("topic", "/camnet/cam_0/audio")
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("channels", 1)
        self.declare_parameter("player", "auto")

        self.topic = str(self.get_parameter("topic").value)
        self.sample_rate = int(self.get_parameter("sample_rate").value)
        self.channels = int(self.get_parameter("channels").value)
        self.player = str(self.get_parameter("player").value).lower()

        self._proc_lock = threading.Lock()
        self._proc: Optional[subprocess.Popen] = None
        self._queue: queue.Queue[bytes] = queue.Queue(maxsize=50)
        self._stop_event = threading.Event()
        self._dropped_chunks = 0

        self._cmd = self._resolve_player_command()
        self._start_player()

        self._writer_thread = threading.Thread(target=self._writer_loop, daemon=True)
        self._writer_thread.start()

        self._sub = self.create_subscription(
            UInt8MultiArray,
            self.topic,
            self._on_audio,
            50,
        )

        self.get_logger().info(
            f"Listening on {self.topic} -> {' '.join(self._cmd)}"
        )

    def _resolve_player_command(self):
        if self.player not in ("auto", "ffplay", "aplay"):
            raise RuntimeError("player must be one of: auto, ffplay, aplay")

        ffplay = shutil.which("ffplay")
        aplay = shutil.which("aplay")

        selected = self.player
        if selected == "auto":
            if ffplay is not None:
                selected = "ffplay"
            elif aplay is not None:
                selected = "aplay"
            else:
                raise RuntimeError("Neither ffplay nor aplay is installed")

        if selected == "ffplay":
            if ffplay is None:
                raise RuntimeError("ffplay not found")
            return [
                ffplay,
                "-loglevel",
                "warning",
                "-fflags",
                "nobuffer",
                "-flags",
                "low_delay",
                "-f",
                "s16le",
                "-ar",
                str(self.sample_rate),
                "-ac",
                str(self.channels),
                "-i",
                "-",
                "-nodisp",
                "-autoexit",
            ]

        if aplay is None:
            raise RuntimeError("aplay not found")
        return [
            aplay,
            "-q",
            "-f",
            "S16_LE",
            "-r",
            str(self.sample_rate),
            "-c",
            str(self.channels),
            "-",
        ]

    def _start_player(self):
        with self._proc_lock:
            self._stop_player_locked()
            self._proc = subprocess.Popen(
                self._cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )

    def _stop_player_locked(self):
        if self._proc is None:
            return
        try:
            if self._proc.stdin is not None:
                self._proc.stdin.close()
        except Exception:
            pass
        self._proc.terminate()
        try:
            self._proc.wait(timeout=1.0)
        except Exception:
            self._proc.kill()
        self._proc = None

    def _restart_player(self):
        try:
            self._start_player()
            self.get_logger().warn("Audio player restarted")
        except Exception as exc:
            self.get_logger().warn(f"Failed to restart audio player: {exc}")

    def _on_audio(self, msg: UInt8MultiArray):
        data = bytes(msg.data)
        if not data:
            return

        try:
            self._queue.put_nowait(data)
            return
        except queue.Full:
            pass

        try:
            _ = self._queue.get_nowait()
            self._queue.put_nowait(data)
            self._dropped_chunks += 1
            if self._dropped_chunks % 20 == 0:
                self.get_logger().warn(
                    f"Dropped {self._dropped_chunks} audio chunks (consumer too slow)"
                )
        except queue.Empty:
            pass

    def _writer_loop(self):
        while not self._stop_event.is_set() and rclpy.ok():
            try:
                data = self._queue.get(timeout=0.2)
            except queue.Empty:
                continue

            with self._proc_lock:
                proc = self._proc

            if proc is None:
                self._restart_player()
                continue
            if proc.poll() is not None:
                self._restart_player()
                continue
            if proc.stdin is None:
                self._restart_player()
                continue

            try:
                proc.stdin.write(data)
            except Exception:
                self._restart_player()

    def destroy_node(self):
        self._stop_event.set()
        try:
            if self._writer_thread.is_alive():
                self._writer_thread.join(timeout=1.0)
        except Exception:
            pass
        with self._proc_lock:
            self._stop_player_locked()
        super().destroy_node()


def main():
    rclpy.init()
    try:
        node = AudioMonitorNode()
    except Exception as exc:
        print(f"audio_monitor init failed: {exc}", file=sys.stderr)
        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
