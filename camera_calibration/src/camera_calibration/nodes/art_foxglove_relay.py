#!/usr/bin/env python3
"""Relay selected ART camera topics from Foxglove WebSocket to local ROS 2."""

from __future__ import annotations

import argparse
from collections import defaultdict
import json
import struct
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CameraInfo, Image
import websocket


SUBPROTOCOL = 'foxglove.sdk.v1'

CAMERA_TOPICS = {
    'stereo': (
        ('/vimba_calib_left/image', '/vimba_calib_left/image', Image),
        ('/vimba_calib_right/image', '/vimba_calib_right/image', Image),
        ('/vimba_calib_left/camera_info',
         '/vimba_calib_left/camera_info', CameraInfo),
        ('/vimba_calib_right/camera_info',
         '/vimba_calib_right/camera_info', CameraInfo),
    ),
    'front': (
        ('/vimba_front/image', '/vimba_front/image', Image),
        ('/vimba_front/camera_info', '/vimba_front/camera_info', CameraInfo),
    ),
    'left': (
        ('/vimba_left/image', '/vimba_left/image', Image),
        ('/vimba_left/camera_info', '/vimba_left/camera_info', CameraInfo),
    ),
    'right': (
        ('/vimba_right/image', '/vimba_right/image', Image),
        ('/vimba_right/camera_info', '/vimba_right/camera_info', CameraInfo),
    ),
    'rear': (
        ('/vimba_rear/image', '/vimba_rear/image', Image),
        ('/vimba_rear/camera_info', '/vimba_rear/camera_info', CameraInfo),
    ),
}


def parse_message_frame(frame: bytes) -> tuple[int, int, bytes]:
    if len(frame) < 13 or frame[0] != 0x01:
        raise ValueError('not a Foxglove message-data frame')
    subscription_id, timestamp_ns = struct.unpack_from('<IQ', frame, 1)
    return subscription_id, timestamp_ns, frame[13:]


class FoxgloveRelay(Node):
    def __init__(self, url: str, camera: str):
        super().__init__('art_foxglove_camera_relay')
        self.url = url
        self.camera = camera
        self.routes = {
            source: {
                'destination': destination,
                'message_type': message_type,
                'publisher': self.create_publisher(
                    message_type, destination, qos_profile_sensor_data),
            }
            for source, destination, message_type in CAMERA_TOPICS[camera]
        }
        self.message_counts = defaultdict(int)
        self._count_lock = threading.Lock()

    def _subscribe_advertisements(
        self,
        socket,
        source: str,
        channels: list[dict],
        subscribed_channel: int | None,
    ) -> int | None:
        route = self.routes[source]
        for channel in channels:
            if channel.get('topic') != source:
                continue
            expected_schema = route['message_type'].__module__.replace(
                '.msg._image', '/msg').replace('.msg._camera_info', '/msg')
            expected_schema += '/' + route['message_type'].__name__
            if channel.get('encoding') != 'cdr':
                self.get_logger().error(
                    f'{source} uses {channel.get("encoding")}, expected cdr')
                continue
            if channel.get('schemaName') != expected_schema:
                self.get_logger().error(
                    f'{source} uses {channel.get("schemaName")}, '
                    f'expected {expected_schema}')
                continue
            channel_id = int(channel['id'])
            if subscribed_channel == channel_id:
                return subscribed_channel
            socket.send(json.dumps({
                'op': 'subscribe',
                'subscriptions': [{'id': 1, 'channelId': channel_id}],
            }))
            self.get_logger().info(f'subscribed via Foxglove: {source}')
            return channel_id
        return subscribed_channel

    def _handle_text(
        self,
        socket,
        source: str,
        message: str,
        subscribed_channel: int | None,
    ) -> int | None:
        data = json.loads(message)
        operation = data.get('op')
        if operation == 'serverInfo':
            self.get_logger().info(
                f'connected to Foxglove Bridge: {data.get("metadata", {})}')
        elif operation == 'advertise':
            subscribed_channel = self._subscribe_advertisements(
                socket, source, data.get('channels', []), subscribed_channel)
        elif operation == 'unadvertise':
            if subscribed_channel in data.get('channelIds', []):
                subscribed_channel = None
        elif operation == 'status' and data.get('level') == 2:
            self.get_logger().error(str(data.get('message', data)))
        return subscribed_channel

    def _handle_binary(self, source: str, frame: bytes) -> None:
        try:
            subscription_id, _timestamp_ns, payload = parse_message_frame(frame)
        except ValueError:
            return
        if subscription_id != 1:
            return
        route = self.routes[source]
        message = deserialize_message(payload, route['message_type'])
        route['publisher'].publish(message)
        with self._count_lock:
            self.message_counts[source] += 1
            count = self.message_counts[source]
        if count == 1:
            if isinstance(message, Image):
                detail = f'{message.width}x{message.height} {message.encoding}'
            else:
                detail = f'{message.width}x{message.height} CameraInfo'
            self.get_logger().info(
                f'relaying {source} -> {route["destination"]}: {detail}')

    def _run_route(self, source: str) -> None:
        while rclpy.ok():
            socket = None
            try:
                subscribed_channel = None
                socket = websocket.create_connection(
                    self.url,
                    subprotocols=[SUBPROTOCOL],
                    timeout=3,
                    http_proxy_host=None,
                    enable_multithread=True,
                )
                socket.settimeout(1.0)
                if socket.getsubprotocol() != SUBPROTOCOL:
                    raise RuntimeError('Foxglove subprotocol negotiation failed')
                while rclpy.ok():
                    try:
                        message = socket.recv()
                    except websocket.WebSocketTimeoutException:
                        continue
                    if not message:
                        raise ConnectionError('Foxglove Bridge closed the connection')
                    if isinstance(message, str):
                        subscribed_channel = self._handle_text(
                            socket, source, message, subscribed_channel)
                    else:
                        self._handle_binary(source, message)
            except KeyboardInterrupt:
                return
            except Exception as exc:
                if rclpy.ok():
                    self.get_logger().error(f'Foxglove relay error: {exc}; retrying')
                    time.sleep(1.0)
            finally:
                if socket is not None:
                    socket.close()

    def run(self) -> None:
        threads = [
            threading.Thread(
                target=self._run_route,
                args=(source,),
                name=f'foxglove-{self.camera}-{source.rsplit("/", 1)[-1]}',
                daemon=True,
            )
            for source in self.routes
        ]
        for thread in threads:
            thread.start()
        while rclpy.ok() and any(thread.is_alive() for thread in threads):
            time.sleep(0.2)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Relay ART camera CDR messages from the vehicle Foxglove Bridge.')
    parser.add_argument('--url', default='ws://10.42.27.200:8765/')
    parser.add_argument('--camera', choices=sorted(CAMERA_TOPICS), default='stereo')
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    rclpy.init(args=None)
    node = FoxgloveRelay(args.url, args.camera)
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
