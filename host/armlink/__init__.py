"""Host driver for the robotic arm controller (arm-fw protocol 1).

    from armlink import connect
    arm = connect()            # auto-detect port
    arm.enable(True)
    arm.jog("J1", 30, feed=0.5)

Or, with no hardware attached:

    arm = connect(simulate=True)
"""
from .link import ArmLink, Reply, find_port, list_serial_ports, parse_data_line
from .arm import Arm, ArmError, ArmState, JointState

__all__ = ["ArmLink", "Reply", "Arm", "ArmError", "ArmState", "JointState",
           "connect", "find_port", "list_serial_ports", "parse_data_line"]


def connect(port=None, baud=115200, simulate=False, sim_present=None, settle: float = 2.0) -> Arm:
    if simulate:
        from .fake import FakeSerial
        return Arm(ArmLink("sim", transport=FakeSerial(sim_present)))
    return Arm(ArmLink(find_port(port), baud, settle=settle))
