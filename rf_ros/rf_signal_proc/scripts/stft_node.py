#!/usr/bin/env python3

from __future__ import annotations

import math
from typing import Optional

import numpy as np
from rclpy.node import Node
import rclpy
from rf_msgs.msg import IqFrame, StftFrame
from scipy import signal


class StftNode(Node):
    def __init__(self) -> None:
        super().__init__("stft_node")

        # Fallback values used until the first IqFrame arrives with metadata.
        self._fs_hz = None
        self._cf_hz = None

        self._buffer_size = max(int(self.declare_parameter("buffer_size", 100000).value), 2)
        self._fft_size = max(int(self.declare_parameter("fft_size", 2048).value), 2)
        self._hop_size = max(int(self.declare_parameter("hop_size", 512).value), 1)
        self._win_size = max(int(self.declare_parameter("win_size", 2048).value),2)

        iq_topic = "iq"
        stft_topic = "stft"

        self._iq_buffer = np.empty(self._buffer_size, dtype=np.complex128)
        self._nsamples = 0

        self._pub = self.create_publisher(StftFrame, stft_topic, 10)
        self._sub = self.create_subscription(
            IqFrame, iq_topic, self._on_iq, 10
        )

        self.get_logger().info(
            "StftNode ready: "
            f"buf={self._buffer_size} samp  win={self._win_size}  "
            f"fft={self._fft_size}  hop={self._hop_size}"
        )

    def _on_iq(self, msg: IqFrame) -> None:
        # Update fs/fc from message metadata; recompute buffer size if fs changed.
        if self._fs_hz != msg.fs_hz or self._cf_hz != msg.fc_hz:
            self._fs_hz = msg.fs_hz
            self._cf_hz = msg.fc_hz
            self._nsamples = 0

        raw = np.asarray(msg.data, dtype=np.float64)
        if raw.size % 2 != 0:
            self.get_logger().warn(
                f"Dropping IQ frame: odd element count ({raw.size})"
            )
            return
        iq = raw.reshape(-1, 2)

        samples_read = 0
        iq_len = iq.shape[0]
        while samples_read < iq_len:
            samples_to_read = min(self._buffer_size - self._nsamples, iq_len - samples_read)
            self._iq_buffer[self._nsamples:self._nsamples+samples_to_read] = \
                                iq[samples_read:samples_read+samples_to_read, 0] + 1j * iq[samples_read:samples_read+samples_to_read, 1]
            self._nsamples += samples_to_read
            samples_read += samples_to_read
            if self._nsamples == self._buffer_size:
                self._process_buffer()
                self._nsamples = 0

    def _process_buffer(self) -> None:

        sxx, f_hz, t_s = self._compute_stft(
            iq=self._iq_buffer,
            fs=self._fs_hz,
            win_size=self._win_size,
            hop=self._hop_size,
            nfft=self._fft_size
        )

        if sxx.size == 0 or f_hz.size == 0 or t_s.size == 0:
            return

        sxx_db = 10.0 * np.log10(np.maximum(sxx, 1e-30))

        msg = StftFrame()
        msg.stamp = self.get_clock().now().to_msg()
        msg.fs_hz = self._fs_hz
        msg.num_frames = int(sxx_db.shape[1])
        msg.fft_size = int(sxx_db.shape[0])
        msg.waterfall_db = sxx_db.T.astype(np.float64, copy=False).reshape(-1).tolist()
        msg.freq_ghz = ((f_hz + self._cf_hz) / 1e9).astype(np.float64).tolist()
        msg.time_s = np.asarray(t_s, dtype=np.float64).tolist()
        self._pub.publish(msg)

    @staticmethod
    def _compute_stft(
        iq: np.ndarray,
        fs: float,
        win_size: int,
        hop: int,
        nfft: int,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        
        if hasattr(signal, "ShortTimeFFT"):
            win = signal.get_window("hann", win_size, fftbins=True)
            stft_obj = signal.ShortTimeFFT(
                win=win,
                hop=hop,
                fs=fs,
                fft_mode="centered",
                mfft=nfft,
                scale_to="psd",
            )
            sxx = stft_obj.spectrogram(iq)
            return sxx, stft_obj.f, stft_obj.t(len(iq))

        noverlap = win_size - hop
        f_hz, t_s, sxx = signal.spectrogram(
            iq,
            fs=fs,
            window=signal.get_window("hann", win_size),
            nperseg=win_size,
            noverlap=noverlap,
            nfft=nfft,
            return_onesided=False,
            scaling="density",
        )
        f_hz = np.fft.fftshift(f_hz)
        sxx = np.fft.fftshift(sxx, axes=0)
        return sxx, f_hz, t_s


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = StftNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
