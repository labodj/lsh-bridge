"""Cross-check committed v3 examples against the schema and an independent codec."""

import json
import unittest
from pathlib import Path

import msgpack
from jsonschema.validators import validator_for


class WireContract(unittest.TestCase):
    def test_golden_payloads_and_wire_vectors(self):
        shared = Path(__file__).resolve().parents[1] / "shared"
        golden = json.loads((shared / "lsh_protocol_golden_payloads.json").read_text())
        schema = json.loads((shared / "lsh_protocol.schema.json").read_text())
        validator = validator_for(schema)
        validator.check_schema(schema)
        validate = validator(schema).validate
        for name, payload in golden["payloads"].items():
            with self.subTest(example=name):
                validate(payload)
        self.assertTrue(golden["wireVectors"])
        for name, vector in golden["wireVectors"].items():
            with self.subTest(vector=name):
                payload = vector["payload"]
                validate(payload)
                raw = bytes.fromhex(vector["msgpack"])
                self.assertEqual(msgpack.unpackb(raw), payload)
                self.assertEqual(msgpack.packb(payload), raw)
                escaped = raw.replace(b"\xdb", b"\xdb\xdd").replace(b"\xc0", b"\xdb\xdc")
                self.assertEqual(bytes.fromhex(vector["uart"]), b"\xc0" + escaped + b"\xc0")
