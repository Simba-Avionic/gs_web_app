import json
import sys
import tempfile
import textwrap
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


from mavlink.xml_to_json import convert_xml_to_json
from shared.utils import get_type_mapping


class XmlToJsonTests(unittest.TestCase):
    def test_mavlink_array_types_use_base_ros_type(self):
        self.assertEqual(get_type_mapping("uint8_t[3]"), "uint8")
        self.assertEqual(get_type_mapping("float[4]"), "float32")

    def test_outbound_messages_are_skipped_and_ack_is_kept(self):
        xml = textwrap.dedent(
            """\
            <mavlink>
              <messages>
                <message id="70" name="SIMBA_TANK_PRESSURE">
                  <field type="uint16_t" name="pressure" />
                </message>
                <message id="74" name="SIMBA_GS_HEARTBEAT">
                  <field type="uint64_t" name="timestamp" />
                  <field type="uint16_t" name="values" />
                </message>
                <message id="147" name="SIMBA_ACTUATOR_CMD">
                  <field type="uint8_t" name="actuator_id" />
                  <field type="uint8_t" name="value" />
                </message>
                <message id="148" name="SIMBA_TANKING_COMMAND_ACK">
                  <field type="uint64_t" name="timestamp" />
                  <field type="uint64_t" name="command_timestamp" />
                  <field type="uint16_t" name="command" />
                  <field type="uint8_t" name="status" />
                </message>
                <message id="77" name="SIMBA_COMPUTERS_TELEMETRY">
                  <field type="uint8_t[3]" name="mb_temperature" />
                </message>
              </messages>
            </mavlink>
            """
        )

        with tempfile.NamedTemporaryFile("w", suffix=".xml") as xml_file:
            with tempfile.NamedTemporaryFile("r", suffix=".json") as json_file:
                xml_file.write(xml)
                xml_file.flush()

                convert_xml_to_json(xml_file.name, json_file.name)
                data = json.load(json_file)

        topics = {topic["topic_name"]: topic for topic in data["topics"]}

        self.assertNotIn("mavlink/simba_tank_pressure", topics)
        self.assertNotIn("mavlink/simba_gs_heartbeat", topics)
        self.assertNotIn("mavlink/simba_actuator_cmd", topics)
        self.assertIn("mavlink/simba_tanking_command_ack", topics)
        self.assertIn("mavlink/simba_computers_telemetry", topics)

        computer_fields = {
            field["val_name"]: field["type"]
            for field in topics["mavlink/simba_computers_telemetry"]["msg_fields"]
        }
        self.assertEqual(computer_fields["mb_temperature"], "uint8")


if __name__ == "__main__":
    unittest.main()
