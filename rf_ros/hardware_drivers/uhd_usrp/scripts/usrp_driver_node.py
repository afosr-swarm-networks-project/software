#!/usr/bin/env python3

from __future__ import annotations

import threading

import numpy as np
import rclpy
from rclpy.node import Node

import uhd

from rf_msgs.msg import IqFrame

class UsrpDriverNode(Node):
    def __init__(self) -> None:
        super().__init__("usrp_driver_node")

        self._device_args = str(self.declare_parameter("device_args", "").value)
        self._freq_hz = float(self.declare_parameter("freq_hz", 2.4e9).value)
        self._rate_hz = float(self.declare_parameter("rate_hz", 1e6).value)
        self._gain_db = float(self.declare_parameter("gain_db", 10.0).value)
        self._channel = int(self.declare_parameter("channel", 0).value)
        self._chunk = int(self.declare_parameter("samples_per_chunk", 1024).value)

        self._pub = self.create_publisher(IqFrame, "iq", 10)

        self._thread = threading.Thread(target=self._stream_worker, daemon=True)
        self._thread.start()

    def _stream_worker(self) -> None:
        usrp = uhd.usrp.MultiUSRP(self._device_args)
        usrp.set_rx_rate(self._rate_hz, self._channel)
        usrp.set_rx_freq(uhd.types.TuneRequest(self._freq_hz), self._channel)
        usrp.set_rx_gain(self._gain_db, self._channel)

        fs_hz = float(usrp.get_rx_rate(self._channel))
        fc_hz = float(usrp.get_rx_freq(self._channel))

        st_args = uhd.usrp.StreamArgs("fc32", "sc16")
        st_args.channels = [self._channel]
        streamer = usrp.get_rx_stream(st_args)

        metadata = uhd.types.RXMetadata()
        buffer_samps = streamer.get_max_num_samps()
        samples = np.empty((1, self._chunk), dtype=np.complex64)
        recv_buffer = np.zeros((1, buffer_samps), dtype=np.complex64)

        self.get_logger().info(
            f"USRP driver started: fs={fs_hz / 1e6:.2f} MHz  "
            f"fc={fc_hz / 1e9:.4f} GHz  gain={self._gain_db:.1f} dB  "
            f"chunk={self._chunk} samp"
        )

        stream_cmd = uhd.types.StreamCMD(uhd.types.StreamMode.start_cont)
        stream_cmd.stream_now = True
        streamer.issue_stream_cmd(stream_cmd)

        while True:
            recv_samps = 0
            while recv_samps < self._chunk:
                samps = streamer.recv(recv_buffer, metadata)
                if metadata.error_code != uhd.types.RXMetadataErrorCode.none:
                    print(metadata.strerror())
                if samps:
                    real_samps = min(self._chunk - recv_samps, samps)
                    samples[:, recv_samps : recv_samps + real_samps] = recv_buffer[:, 0:real_samps]
                    recv_samps += real_samps

            try:
                self._publish(samples[self._channel], fs_hz, fc_hz)
            except Exception:
                break

        stop_cmd = uhd.types.StreamCMD(uhd.types.StreamMode.stop_cont)
        streamer.issue_stream_cmd(stop_cmd)
        

    def _publish(self, samples: np.ndarray, fs_hz: float, fc_hz: float) -> bool:

        iq = np.empty(2 * len(samples), dtype=np.float32)
        iq[0::2] = samples.real
        iq[1::2] = samples.imag

        msg = IqFrame()
        msg.stamp = self.get_clock().now().to_msg()
        msg.data = iq.tolist()
        msg.fs_hz = fs_hz
        msg.fc_hz = fc_hz
        self._pub.publish(msg)

    def destroy_node(self) -> None:
        self._thread.join(timeout=3.0)
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = UsrpDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
