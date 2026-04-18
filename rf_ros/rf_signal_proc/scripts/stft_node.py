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
        self._fs_hz = float(self.declare_parameter("fs_hz", 1000.0).value)
        self._center_freq_hz = float(
            self.declare_parameter("center_freq_hz", 2.4e9).value
        )
        self._stft_win_s = float(self.declare_parameter("stft_win_s", 0.04915).value)
        self._fft_size = max(int(self.declare_parameter("fft_size", 2048).value), 2)
        self._hop_size = max(
            int(self.declare_parameter("hop_size", 512).value), 1
        )
        iq_topic = "iq"
        stft_topic = "stft"

        self._buffer_size = max(int(self._fs_hz * self._stft_win_s), self._fft_size)
        self._iq_buffer = np.empty(0, dtype=np.complex128)

        self._pub = self.create_publisher(StftFrame, stft_topic, 10)
        self._sub = self.create_subscription(
            IqFrame, iq_topic, self._on_iq, 10
        )

        self.get_logger().info(
            "StftNode ready: "
            f"fs={self._fs_hz:.0f} Hz  fc={self._center_freq_hz:.0f} Hz  "
            f"win={self._stft_win_s:.5f} s ({self._buffer_size} samp)  "
            f"fft={self._fft_size}  hop={self._hop_size}  "
        )

    def _on_iq(self, msg: IqFrame) -> None:
        # Update fs/fc from message metadata; recompute buffer size if fs changed.
        if msg.fs_hz > 0.0 and msg.fs_hz != self._fs_hz:
            self._fs_hz = msg.fs_hz
            self._buffer_size = max(
                int(self._fs_hz * self._stft_win_s), self._fft_size
            )
        if msg.fc_hz > 0.0:
            self._center_freq_hz = msg.fc_hz

        data = np.asarray(msg.data, dtype=np.float64)
        if data.size < 2:
            return

        pair_count = data.size // 2
        iq = data[: 2 * pair_count].reshape(-1, 2)
        samples = iq[:, 0] + 1j * iq[:, 1]

        if self._iq_buffer.size == 0:
            self._iq_buffer = samples.astype(np.complex128, copy=True)
        else:
            self._iq_buffer = np.concatenate(
                [self._iq_buffer, samples.astype(np.complex128, copy=False)]
            )

        if self._iq_buffer.size >= self._buffer_size:
            self._process_buffer()

    def _process_buffer(self) -> None:
        iq = self._iq_buffer[:self._buffer_size]
        self._iq_buffer = self._iq_buffer[self._buffer_size:]

        sxx, f_hz, t_s = self._compute_stft(
            iq=iq,
            fs=self._fs_hz,
            nperwindow=self._fft_size,
            hop=self._hop_size,
            nfft=self._fft_size,
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
        msg.freq_ghz = ((f_hz + self._center_freq_hz) / 1e9).astype(np.float64).tolist()
        msg.time_s = np.asarray(t_s, dtype=np.float64).tolist()
        self._pub.publish(msg)

    @staticmethod
    def _compute_stft(
        iq: np.ndarray,
        fs: float,
        nperwindow: Optional[int] = None,
        hop: Optional[int] = None,
        nfft: Optional[int] = None,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        n_samples = len(iq)
        nperwindow = nperwindow or min(256, max(2, n_samples // 4))
        hop = hop or max(1, nperwindow // 8)
        nfft = nfft or int(2 ** math.ceil(math.log2(max(nperwindow * 4, 2))))

        if hasattr(signal, "ShortTimeFFT"):
            win = signal.get_window("hann", nperwindow, fftbins=True)
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

        noverlap = nperwindow - hop
        f_hz, t_s, sxx = signal.spectrogram(
            iq,
            fs=fs,
            window=signal.get_window("hann", nperwindow),
            nperseg=nperwindow,
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
