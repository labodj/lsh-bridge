#!/usr/bin/env python3
# /// script
# requires-python = ">=3.11"
# dependencies = ["paho-mqtt==2.1.0"]
# ///
"""Observe bridge health, optionally sending bounded invalid Homie boolean writes.

No valid actuator commands, retained publications, OTA or device resets are sent.
Retained samples are reported separately: they are not proof of current liveness.
Run with uv to use an isolated Python environment; --help needs only Python.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import sys
import time
from collections import deque
from pathlib import Path
from threading import Condition

STATS = (
    "uptime",
    "freeheap",
    "mqttinbounddropped",
    "mqttackdropped",
    "mqttinboundmaxdepth",
    "mqttackmaxdepth",
)


def identifier(value: str) -> str:
    if len(value) > 128 or not re.fullmatch(r"[a-z0-9][a-z0-9-]*", value):
        raise argparse.ArgumentTypeError(
            "use a Homie id of at most 128 characters, without slashes or MQTT wildcards"
        )
    return value


def seconds(value: str) -> float:
    result = float(value)
    if not math.isfinite(result) or not 0 < result <= 3600:
        raise argparse.ArgumentTypeError(
            "seconds must be finite, greater than 0 and at most 3600"
        )
    return result


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("devices", nargs="+", type=identifier, metavar="DEVICE")
    parser.add_argument(
        "--config", type=Path, help="generated bridge-ota.json for broker defaults"
    )
    parser.add_argument("--broker-host", help="required unless set in --config")
    parser.add_argument("--broker-port", type=int)
    parser.add_argument("--username")
    parser.add_argument(
        "--password-env", help="environment variable containing the MQTT password"
    )
    parser.add_argument("--homie-base", help="Homie topic prefix (default: homie/5)")
    parser.add_argument(
        "--duration", type=seconds, default=30, help="observation seconds (default: 30)"
    )
    parser.add_argument(
        "--timeout", type=seconds, default=10, help="connection/ACK timeout seconds"
    )
    parser.add_argument(
        "--burst",
        type=int,
        default=0,
        metavar="COUNT",
        help="opt in to 1-1000 invalid writes to one explicit --node",
    )
    parser.add_argument(
        "--node", type=identifier, help="single actuator node for --burst"
    )
    parser.add_argument(
        "--protected-device",
        action="append",
        default=[],
        type=identifier,
        help="deny burst to this device (repeatable, installation policy)",
    )
    parser.add_argument(
        "--allow-protected", action="store_true", help="explicitly override that policy"
    )
    args = parser.parse_args(argv)
    args.devices = list(dict.fromkeys(args.devices))
    if len(args.devices) > 32 or not 0 <= args.burst <= 1000:
        parser.error("at most 32 devices and 1000 burst messages are supported")
    if args.burst and (not args.node or len(args.devices) != 1):
        parser.error("--burst requires exactly one DEVICE and --node")
    if args.node and not args.burst:
        parser.error("--node requires --burst")
    if (
        args.burst
        and set(args.devices).intersection(args.protected_device)
        and not args.allow_protected
    ):
        parser.error(
            "burst target is protected; use --allow-protected only after checking the target"
        )
    return args


def connection_settings(args: argparse.Namespace) -> tuple[dict, str]:
    raw = json.loads(args.config.read_text(encoding="utf-8")) if args.config else {}
    if (
        not isinstance(raw, dict)
        or raw.get("schema", "homie-ota-config/v1") != "homie-ota-config/v1"
    ):
        raise ValueError("expected a generated homie-ota-config/v1 config")
    broker, homie = raw.get("broker", {}), raw.get("homie", {})
    if not isinstance(broker, dict) or not isinstance(homie, dict):
        raise ValueError("broker and homie config must be objects")  # noqa: TRY004 - invalid JSON config.
    broker = dict(broker)
    for option, value in (
        ("host", args.broker_host),
        ("port", args.broker_port),
        ("username", args.username),
        ("password_env", args.password_env),
    ):
        if value is not None:
            broker[option] = value
    if args.username is not None:
        broker.pop("username_env", None)
    for key in ("username", "password"):
        env_name = broker.get(key + "_env")
        if env_name:
            if not isinstance(env_name, str) or env_name not in os.environ:
                raise ValueError(
                    f"set the configured {key} environment variable before connecting"
                )
            broker[key] = os.environ[env_name]
    if not isinstance(broker.get("host"), str) or not broker["host"].strip():
        raise ValueError("provide --broker-host or a broker host in --config")
    broker.setdefault("port", 1883)
    if type(broker["port"]) is not int or not 1 <= broker["port"] <= 65535:
        raise ValueError("MQTT port must be between 1 and 65535")
    for key in ("username", "password", "tls_cacert", "tls_certfile", "tls_keyfile"):
        if broker.get(key) is not None and not isinstance(broker[key], str):
            raise ValueError(f"broker {key} must be a string")
    if broker.get("password") is not None and broker.get("username") is None:
        raise ValueError("a MQTT password requires a username")
    base = args.homie_base or homie.get("base_topic", "homie/5")
    if (
        not isinstance(base, str)
        or not base.strip("/")
        or len(base.encode("utf-8")) > 1024
        or any(c in base for c in ("+", "#", "\0", "\n", "\r"))
    ):
        raise ValueError("Homie base must be a literal, nonempty MQTT prefix")
    if broker.get("tls_insecure"):
        raise ValueError("insecure TLS is not supported by this diagnostic tool")
    return broker, base.strip("/")


class Monitor:
    """Bounded topic cache with callback failures propagated to the calling thread."""

    def __init__(self, args: argparse.Namespace, broker: dict, base: str) -> None:
        import paho.mqtt.client as mqtt
        from paho.mqtt.enums import CallbackAPIVersion

        self.args, self.broker = args, broker
        self.cv = Condition()
        self.values: dict[str, tuple[str, bool]] = {}
        self.issues: deque[str] = deque(maxlen=20)
        self.error: str | None = None
        self.subscribed = self.closing = False
        self.prefixes = [f"{base}/{device}" for device in args.devices]
        self.topics = [
            f"{prefix}/{suffix}"
            for prefix in self.prefixes
            for suffix in (
                "$state",
                "$implementation/reset-reason",
                *("$stats/" + name for name in STATS),
            )
        ]
        if args.burst:
            self.topics.extend(
                f"{self.prefixes[0]}/{args.node}/state/{key}"
                for key in ("$datatype", "$settable")
            )
        self.client = mqtt.Client(
            CallbackAPIVersion.VERSION2, reconnect_on_failure=False
        )
        self.client.connect_timeout = args.timeout
        self.client.max_queued_messages_set(20)
        if broker.get("username") is not None:
            self.client.username_pw_set(broker["username"], broker.get("password"))
        if any(
            broker.get(key) for key in ("tls_cacert", "tls_certfile", "tls_keyfile")
        ):
            self.client.tls_set(
                ca_certs=broker.get("tls_cacert"),
                certfile=broker.get("tls_certfile"),
                keyfile=broker.get("tls_keyfile"),
            )
        self.client.on_connect = self._on_connect
        self.client.on_subscribe = self._on_subscribe
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message

    def _fail(self, message: str) -> None:
        with self.cv:
            self.error = message
            self.cv.notify_all()

    def _on_connect(self, client, _userdata, _flags, reason, _properties) -> None:
        if reason.is_failure:
            self._fail(f"MQTT connection rejected: {reason}")
            return
        result, _mid = client.subscribe([(topic, 1) for topic in self.topics])
        if result != 0:
            self._fail(f"MQTT subscribe failed: {result}")

    def _on_subscribe(self, _client, _userdata, _mid, reasons, _properties) -> None:
        with self.cv:
            if not reasons or any(reason.is_failure for reason in reasons):
                self.error = "broker rejected a diagnostic subscription"
            else:
                self.subscribed = True
            self.cv.notify_all()

    def _on_disconnect(self, _client, _userdata, _flags, reason, _properties) -> None:
        if not self.closing:
            self._fail(f"MQTT disconnected during observation: {reason}")

    def _on_message(self, _client, _userdata, message) -> None:
        if message.topic not in self.topics:
            return
        value = message.payload.decode("utf-8", errors="replace")[:256]
        with self.cv:
            previous = self.values.get(message.topic)
            if previous:
                old = previous[0]
                metric = message.topic.rsplit("/", 1)[-1]
                if metric == "$state" and old == "ready" and value != "ready":
                    self.issues.append(f"{message.topic}: ready -> {json.dumps(value)}")
                if (
                    old.isascii()
                    and old.isdigit()
                    and value.isascii()
                    and value.isdigit()
                ):
                    if metric == "uptime" and int(value) < int(old):
                        self.issues.append(
                            f"{message.topic}: uptime decreased (restart suspected)"
                        )
                    if metric.endswith("dropped") and int(value) > int(old):
                        self.issues.append(
                            f"{message.topic}: increased from {old} to {value}"
                        )
            self.values[message.topic] = (value, bool(message.retain))
            self.cv.notify_all()

    def wait(self, predicate, description: str, timeout: float | None = None) -> None:
        deadline = time.monotonic() + (
            self.args.timeout if timeout is None else timeout
        )
        with self.cv:
            while not predicate():
                if self.error:
                    raise RuntimeError(self.error)
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(f"timed out waiting for {description}")
                self.cv.wait(remaining)
            if self.error:
                raise RuntimeError(self.error)

    def __enter__(self):
        try:
            self.client.connect(self.broker["host"], self.broker["port"], keepalive=30)
            self.client.loop_start()
            self.wait(lambda: self.subscribed, "MQTT subscriptions")
            return self
        except BaseException:
            self.close()
            raise

    def close(self) -> None:
        self.closing = True
        self.client.disconnect()
        self.client.loop_stop()

    def __exit__(self, *_exc) -> None:
        self.close()

    def burst(self) -> None:
        """Only literal non-boolean payloads; never replay a captured actuator state."""
        prefix = self.prefixes[0]
        node = f"{prefix}/{self.args.node}/state"
        self.wait(
            lambda: (
                self.values.get(f"{prefix}/$state", (None,))[0] == "ready"
                and self.values.get(node + "/$datatype", (None,))[0] == "boolean"
                and self.values.get(node + "/$settable", (None,))[0] == "true"
            ),
            "ready device with a settable boolean node",
        )
        for sequence in range(self.args.burst):
            with self.cv:
                if self.error or self.issues:
                    raise RuntimeError(self.error or self.issues[-1])
            info = self.client.publish(
                node + "/set", f"invalid-bool-{sequence}", qos=1, retain=False
            )
            if info.rc != 0:
                raise RuntimeError(f"MQTT publish rejected: {info.rc}")
            info.wait_for_publish(timeout=self.args.timeout)
            if not info.is_published():
                raise TimeoutError("MQTT publish ACK timeout")
        print(
            f"Published {self.args.burst} invalid boolean payloads; broker ACKs received."
        )

    def observe(self) -> None:
        deadline = time.monotonic() + self.args.duration
        self.wait(
            lambda: time.monotonic() >= deadline,
            "observation",
            self.args.duration + 0.1,
        )

    def report(self) -> int:
        with self.cv:
            for prefix in self.prefixes:
                print(prefix)
                for topic in self.topics:
                    if not topic.startswith(prefix + "/"):
                        continue
                    value, retained = self.values.get(topic, ("missing", True))
                    print(
                        f"  {topic[len(prefix) + 1 :]}: {json.dumps(value)} "
                        f"({'retained/unknown' if retained else 'live'})"
                    )
                if self.values.get(prefix + "/$state", (None,))[0] != "ready":
                    self.issues.append(f"{prefix}: ready state not observed")
            for issue in self.issues:
                print("issue: " + issue)
            print(
                "Observed MQTT data only; this does not verify physical actuator operation."
            )
            return 1 if self.issues else 0


def main(argv=None) -> int:
    args = parse_args(argv)
    try:
        broker, base = connection_settings(args)
        with Monitor(args, broker, base) as monitor:
            if args.burst:
                monitor.burst()
            monitor.observe()
            return monitor.report()
    except ImportError:
        print("Missing Paho MQTT 2.1: run this script with uv run.", file=sys.stderr)
    except (OSError, ValueError, RuntimeError) as exc:
        print(f"diagnostic failed: {exc}", file=sys.stderr)
    except KeyboardInterrupt:
        return 130
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
