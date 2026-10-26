"""Offline safety checks and a real, loopback-only MQTT round trip."""

from __future__ import annotations

import contextlib
import io
import json
import os
import shutil
import socket
import subprocess
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import mqtt_diagnostics as diagnostics
import paho.mqtt.client as mqtt


class DiagnosticsTest(unittest.TestCase):
    def test_rejects_ambiguous_or_unsafe_arguments(self):
        cases = (
            ["#"],
            ["x" * 129],
            ["j1", "--burst", "1"],
            ["j1", "j2", "--burst", "1", "--node", "1"],
            ["j1", "--burst", "1001", "--node", "1"],
            ["j1", "--duration", "nan"],
            ["j1", "--timeout", "-1"],
            ["j1", "--node", "1"],
            ["j1", "--burst", "1", "--node", "1", "--protected-device", "j1"],
        )
        for argv in cases:
            with (
                self.subTest(argv=argv),
                contextlib.redirect_stderr(io.StringIO()),
                self.assertRaises(SystemExit),
            ):
                diagnostics.parse_args(argv)

    def test_config_credentials_and_overrides(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "ota.json"
            path.write_text(
                json.dumps(
                    {
                        "schema": "homie-ota-config/v1",
                        "broker": {
                            "host": "localhost",
                            "username_env": "NONEXISTENT_TEST_USER",
                            "port": 1883,
                            "password_env": "TEST_MQTT_PASSWORD",
                        },
                        "homie": {"base_topic": "test/homie/"},
                    }
                )
            )
            args = diagnostics.parse_args(
                ["panel", "--config", str(path), "--username", "explicit"]
            )
            with patch.dict(os.environ, {"TEST_MQTT_PASSWORD": "secret"}):
                settings, base = diagnostics.connection_settings(args)
            self.assertEqual(base, "test/homie")
            self.assertEqual(settings["username"], "explicit")
            self.assertEqual(settings["password"], "secret")
            args.homie_base = "homie/#"
            with self.assertRaises(ValueError):
                diagnostics.connection_settings(args)

    def test_callbacks_and_unacknowledged_publish_fail_on_main_thread(self):
        args = diagnostics.parse_args(["panel", "--burst", "1", "--node", "1"])
        monitor = diagnostics.Monitor(
            args, {"host": "localhost", "port": 1883}, "test/homie"
        )
        monitor._on_subscribe(None, None, 1, [SimpleNamespace(is_failure=True)], None)
        with self.assertRaisesRegex(RuntimeError, "rejected"):
            monitor.wait(lambda: False, "subscription")
        monitor.error = None
        monitor.values = {
            "test/homie/panel/$state": ("ready", True),
            "test/homie/panel/1/state/$datatype": ("boolean", True),
            "test/homie/panel/1/state/$settable": ("true", True),
        }
        info = Mock(rc=0)
        info.is_published.return_value = False
        monitor.client = Mock()
        monitor.client.publish.return_value = info
        with self.assertRaisesRegex(TimeoutError, "ACK timeout"):
            monitor.burst()
        monitor.client.publish.assert_called_once_with(
            "test/homie/panel/1/state/set", "invalid-bool-0", qos=1, retain=False
        )
        monitor._on_disconnect(None, None, None, "connection lost", None)
        with self.assertRaisesRegex(RuntimeError, "disconnected"):
            monitor.wait(lambda: True, "anything")

    def test_failed_enter_always_cleans_up(self):
        args = diagnostics.parse_args(["panel", "--timeout", "0.01"])
        monitor = diagnostics.Monitor(
            args, {"host": "localhost", "port": 1883}, "test/homie"
        )
        monitor.client = Mock()
        with self.assertRaises(TimeoutError), monitor:
            self.fail("subscription did not complete")
        monitor.client.disconnect.assert_called_once()
        monitor.client.loop_stop.assert_called_once()


@unittest.skipUnless(
    shutil.which("mosquitto"), "mosquitto is required for loopback integration"
)
class BrokerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with socket.socket() as reservation:
            reservation.bind(("127.0.0.1", 0))
            cls.port = reservation.getsockname()[1]
        cls.broker = subprocess.Popen(
            [shutil.which("mosquitto"), "-p", str(cls.port)],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        cls.addClassCleanup(cls.stop_broker)
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            if cls.broker.poll() is not None:
                raise RuntimeError("isolated mosquitto failed to start")
            try:
                with socket.create_connection(("127.0.0.1", cls.port), timeout=0.1):
                    return
            except OSError:
                time.sleep(0.02)
        raise TimeoutError("isolated broker startup timed out")

    @classmethod
    def stop_broker(cls):
        cls.broker.terminate()
        try:
            cls.broker.wait(timeout=5)
        except subprocess.TimeoutExpired:
            cls.broker.kill()
            cls.broker.wait()

    def test_passive_monitor_and_invalid_burst_with_real_broker(self):
        publisher = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        publisher.connect("127.0.0.1", self.port)
        publisher.loop_start()
        self.addCleanup(publisher.loop_stop)
        self.addCleanup(publisher.disconnect)
        prefix = "lsh-diagnostics-test/homie/panel"
        for suffix, payload in {
            "$state": "ready",
            "$stats/uptime": "100",
            "$stats/freeheap": "90000",
            "$stats/mqttinbounddropped": "0",
            "$stats/mqttackdropped": "0",
            "1/state/$datatype": "boolean",
            "1/state/$settable": "true",
        }.items():
            info = publisher.publish(prefix + "/" + suffix, payload, qos=1, retain=True)
            info.wait_for_publish(timeout=2)
            self.assertTrue(info.is_published())
        args = diagnostics.parse_args(["panel", "--duration", "0.05", "--timeout", "2"])
        broker = {"host": "127.0.0.1", "port": self.port}
        with diagnostics.Monitor(args, broker, "lsh-diagnostics-test/homie") as monitor:
            monitor.wait(
                lambda: prefix + "/$stats/uptime" in monitor.values, "retained stats"
            )
            self.assertEqual(monitor.values[prefix + "/$stats/uptime"], ("100", True))
            with patch.object(
                monitor.client, "publish", wraps=monitor.client.publish
            ) as publish:
                monitor.observe()
                publish.assert_not_called()
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(monitor.report(), 0)
            publisher.publish(prefix + "/$stats/uptime", "2", qos=1)
            monitor.wait(lambda: bool(monitor.issues), "restart detection")
            self.assertIn("restart suspected", monitor.issues[-1])
        args = diagnostics.parse_args(
            ["panel", "--burst", "3", "--node", "1", "--timeout", "2"]
        )
        with diagnostics.Monitor(args, broker, "lsh-diagnostics-test/homie") as monitor:
            with patch.object(
                monitor.client, "publish", wraps=monitor.client.publish
            ) as publish:
                monitor.burst()
            self.assertEqual(publish.call_count, 3)
            self.assertTrue(
                all(
                    call.kwargs == {"qos": 1, "retain": False}
                    for call in publish.call_args_list
                )
            )
            publisher.publish(prefix + "/$stats/mqttackdropped", "1", qos=1)
            monitor.wait(lambda: bool(monitor.issues), "dropped ACK detection")
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(monitor.report(), 1)


if __name__ == "__main__":
    unittest.main()
