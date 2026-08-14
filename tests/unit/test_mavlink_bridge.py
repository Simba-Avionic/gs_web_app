import importlib
import sys
import tempfile
import textwrap
import threading
import types
import unittest
from pathlib import Path
from types import SimpleNamespace


REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


def install_import_stubs():
    loguru = types.ModuleType("loguru")
    loguru.logger = SimpleNamespace(
        info=lambda *args, **kwargs: None,
        error=lambda *args, **kwargs: None,
        warning=lambda *args, **kwargs: None,
    )

    rclpy = types.ModuleType("rclpy")
    rclpy.ok = lambda: False
    rclpy.init = lambda: None
    rclpy.shutdown = lambda: None

    rclpy_node = types.ModuleType("rclpy.node")

    class Node:
        pass

    rclpy_node.Node = Node

    rclpy_executors = types.ModuleType("rclpy.executors")
    rclpy_executors.ExternalShutdownException = type(
        "ExternalShutdownException",
        (Exception,),
        {},
    )

    serial = types.ModuleType("serial")
    serial.SerialException = Exception
    serial_tools = types.ModuleType("serial.tools")
    serial_list_ports = types.ModuleType("serial.tools.list_ports")
    serial_list_ports.comports = lambda: []
    serial.tools = serial_tools
    serial_tools.list_ports = serial_list_ports

    control_panel_reader = types.ModuleType("drivers.control_panel_reader")

    class ControlPanelReader:
        pass

    control_panel_reader.ControlPanelReader = ControlPanelReader

    gs_interfaces = types.ModuleType("gs_interfaces")
    gs_msgs = types.ModuleType("gs_interfaces.msg")
    for name in [
        "LoadCellsTare",
        "RadioStatus",
        "SimbaRocketHeartbeat",
        "SimbaTankingCommandAck",
        "TankingAbort",
        "TankingCommands",
        "TankingSensors",
    ]:
        setattr(gs_msgs, name, type(name, (), {}))
    gs_interfaces.msg = gs_msgs

    mavutil = types.ModuleType("mavlink.src.mavutil")
    mavutil.mavlink = None
    mavutil.mavlink_connection = lambda *args, **kwargs: None

    simba = types.ModuleType("mavlink.src.simba")
    simba.SIMBA_GS_FLAGS_DISARM = 1
    simba.SIMBA_GS_FLAGS_ARM = 2
    simba.SIMBA_GS_FLAGS_LAUNCH = 4
    simba.SIMBA_GS_FLAGS_ABORT = 8
    simba.SIMBA_GS_FLAGS_VENT_VALVE = 16
    simba.SIMBA_GS_FLAGS_DUMP_VALVE = 32
    simba.SIMBA_GS_FLAGS_CAMERAS = 64
    simba.SIMBA_GS_FLAGS_MAIN_VALVE = 128
    simba.SIMBA_GS_FLAGS_FEED_PRESSURIZER_VALVE = 256
    simba.SIMBA_GS_FLAGS_VENT_PRESSURIZER_VALVE = 512
    simba.SIMBA_GS_FLAGS_DECOUPLER_OXIDIZER = 1024
    simba.SIMBA_GS_FLAGS_DECOUPLER_PRESSURIZER = 2048
    simba.SIMBA_GS_FLAGS_TARE_ROCKET = 4096
    simba.SIMBA_GS_FLAGS_TARE_OXIDIZER = 8192
    simba.SIMBA_GS_FLAGS_TARE_PRESSURIZER = 16384
    simba.MAVLink = type("MAVLink", (), {})

    sys.modules["rclpy"] = rclpy
    sys.modules["loguru"] = loguru
    sys.modules["rclpy.node"] = rclpy_node
    sys.modules["rclpy.executors"] = rclpy_executors
    sys.modules["serial"] = serial
    sys.modules["serial.tools"] = serial_tools
    sys.modules["serial.tools.list_ports"] = serial_list_ports
    sys.modules["drivers.control_panel_reader"] = control_panel_reader
    sys.modules["gs_interfaces"] = gs_interfaces
    sys.modules["gs_interfaces.msg"] = gs_msgs
    sys.modules["mavlink.src.mavutil"] = mavutil
    sys.modules["mavlink.src.simba"] = simba


def import_bridge_module():
    install_import_stubs()
    sys.modules.pop("backend.mavlink_bridge", None)
    return importlib.import_module("backend.mavlink_bridge")


class FakePanelReader:
    def __init__(self, rocket_actions, gs_actions):
        self.rocket_actions = rocket_actions
        self.gs_actions = gs_actions

    def get_rocket_actions(self, actions):
        return self.rocket_actions

    def get_gs_actions(self, actions):
        return self.gs_actions


class FakeMav:
    def __init__(self):
        self.heartbeats = []
        self.pressures = []

    def simba_gs_heartbeat_send(self, timestamp, flags):
        self.heartbeats.append((timestamp, flags))

    def simba_tank_pressure_send(self, pressure):
        self.pressures.append(pressure)


class MavlinkBridgeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bridge_module = import_bridge_module()

    def make_bridge(self):
        bridge = self.bridge_module.MavlinkBridge.__new__(
            self.bridge_module.MavlinkBridge
        )
        bridge._panel_lock = threading.Lock()
        bridge._pressure_lock = threading.Lock()
        bridge._last_oxidizer_pressure_bar = None
        bridge._pressure_sender_warned = False
        bridge._missing_gs_flag_warnings = set()
        return bridge

    def test_pressure_scaling_clamps_to_uint16(self):
        bridge = self.make_bridge()

        self.assertEqual(bridge._pressure_bar_to_uint16(12.34), 1234)
        self.assertEqual(bridge._pressure_bar_to_uint16(-1.0), 0)
        self.assertEqual(bridge._pressure_bar_to_uint16(1000.0), 65535)

    def test_tanking_sensor_callback_stores_only_finite_pressure(self):
        bridge = self.make_bridge()

        bridge._on_tanking_sensors(SimpleNamespace(pressure_bar=7.5))
        self.assertEqual(bridge._last_oxidizer_pressure_bar, 7.5)

        bridge._on_tanking_sensors(SimpleNamespace(pressure_bar=float("nan")))
        self.assertEqual(bridge._last_oxidizer_pressure_bar, 7.5)

    def test_send_oxidizer_pressure_uses_scaled_mavlink_message(self):
        bridge = self.make_bridge()
        mav = FakeMav()
        bridge.master = SimpleNamespace(mav=mav)

        bridge._send_oxidizer_pressure(42.42)

        self.assertEqual(mav.pressures, [4242])

    def test_rocket_heartbeat_includes_all_radio_tanking_flags(self):
        module = self.bridge_module
        bridge = self.make_bridge()
        mav = FakeMav()
        bridge.master = SimpleNamespace(mav=mav)
        bridge._control_panel_reader = FakePanelReader(
            rocket_actions={
                ("arm_disarm", "rocket"): 1,
                ("tank_vent", "rocket"): 1,
                ("dump", "rocket"): 1,
                ("enable_cameras", "rocket"): 1,
                ("ignition", "rocket"): 0,
                ("abort", "abort"): 0,
            },
            gs_actions={
                ("valve_feed_oxidizer", "gs"): module.MAIN_VALVE_OPEN_VALUE,
                ("valve_feed_pressurizer", "gs"): 2,
                ("valve_vent_pressurizer", "gs"): 1,
                ("decoupler_oxidizer", "gs"): 1,
                ("decoupler_pressurizer", "gs"): 1,
                ("tare_rocket", "gs"): 1,
                ("tare_oxidizer", "gs"): 1,
                ("tare_pressurizer", "gs"): 1,
            },
        )

        bridge._process_rocket_data({}, is_burst=False)

        expected_flags = (
            module.simba_dialect.SIMBA_GS_FLAGS_ARM
            | module.simba_dialect.SIMBA_GS_FLAGS_VENT_VALVE
            | module.simba_dialect.SIMBA_GS_FLAGS_DUMP_VALVE
            | module.simba_dialect.SIMBA_GS_FLAGS_CAMERAS
            | module.simba_dialect.SIMBA_GS_FLAGS_MAIN_VALVE
            | module.simba_dialect.SIMBA_GS_FLAGS_FEED_PRESSURIZER_VALVE
            | module.simba_dialect.SIMBA_GS_FLAGS_VENT_PRESSURIZER_VALVE
            | module.simba_dialect.SIMBA_GS_FLAGS_DECOUPLER_OXIDIZER
            | module.simba_dialect.SIMBA_GS_FLAGS_DECOUPLER_PRESSURIZER
            | module.simba_dialect.SIMBA_GS_FLAGS_TARE_ROCKET
            | module.simba_dialect.SIMBA_GS_FLAGS_TARE_OXIDIZER
            | module.simba_dialect.SIMBA_GS_FLAGS_TARE_PRESSURIZER
        )

        self.assertEqual(len(mav.heartbeats), 1)
        _, sent_flags = mav.heartbeats[0]
        self.assertEqual(sent_flags, expected_flags)
        self.assertEqual(bridge._last_rocket_flags, expected_flags)

    def test_outbound_mavlink_messages_are_not_published_as_ros_topics(self):
        module = self.bridge_module
        bridge = self.make_bridge()
        created_topics = []

        def create_publisher(msg_type, topic_name, qos):
            created_topics.append(topic_name)
            return SimpleNamespace(msg_type=msg_type)

        bridge.create_publisher = create_publisher

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
                <message id="148" name="SIMBA_TANKING_COMMAND_ACK">
                  <field type="uint64_t" name="timestamp" />
                </message>
              </messages>
            </mavlink>
            """
        )

        with tempfile.NamedTemporaryFile("w", suffix=".xml") as xml_file:
            xml_file.write(xml)
            xml_file.flush()
            publishers = module.MavlinkBridge.create_publishers_from_xml(
                bridge,
                xml_file.name,
            )

        self.assertEqual(created_topics, ["mavlink/simba_tanking_command_ack"])
        self.assertEqual(set(publishers), {"SimbaTankingCommandAck"})


if __name__ == "__main__":
    unittest.main()
