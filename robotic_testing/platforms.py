from software_control.software_control import EC_Lab_Win11, EC_Lab_Win10, OBS_Win, Camera_Win10, TeamViewer, ToDesk
from robotic_testing.common.robot_test import RobotTest_AcidicPlatform, RobotTest_AlkalinePlatform, RobotTest_VLA
from db_control.database import Database
from robotic_testing.common.robotic_arms.xarm6.xarm6 import xArm6
from robotic_testing.common.robotic_arms.xarm7.xarm7 import xArm7


def init_acidic_platform_1318(exp_name, channel_id=2, connect_db=True):
    teamviewer = TeamViewer(exp_name=exp_name)
    todesk = ToDesk(exp_name=exp_name)
    db = Database(exp_name=exp_name) if connect_db else None
    rt = RobotTest_AcidicPlatform(
        exp_name=exp_name,
        ec_lab=EC_Lab_Win11(exp_name=exp_name, channel_id=channel_id),
        camera=OBS_Win(exp_name=exp_name),
        db=db,
        arm=xArm6(exp_name=exp_name),
        prompt_window_bypass_list=[
            teamviewer.bypass_side_window,
            teamviewer.bypass_session_end_prompt_window,
            todesk.bypass_side_window,
        ]
    )
    return rt


def init_alkaline_platform_270(exp_name, connect_db=True):
    teamviewer = TeamViewer(exp_name=exp_name)
    todesk = ToDesk(exp_name=exp_name)
    db = Database(exp_name=exp_name) if connect_db else None
    rt = RobotTest_AlkalinePlatform(
        exp_name=exp_name,
        ec_lab=EC_Lab_Win10(exp_name=exp_name),
        camera=Camera_Win10(exp_name=exp_name),
        db=db,
        arm=xArm7(exp_name=exp_name),
        prompt_window_bypass_list=[
            teamviewer.bypass_side_window,
            teamviewer.bypass_session_end_prompt_window,
            todesk.bypass_side_window,
        ]
    )
    return rt


def init_vla_platform(exp_name, arm='xarm7', connect_db=False, to_log=True):
    """
    Arm-only session for free-form / VLA-driven control.

    Skips EC-Lab, the camera and the prompt-window bypasses, none of which are needed to
    drive the arm from a text instruction. Use one of the platform initialisers above when
    you want to run an actual electrochemical test.

    :param arm: which arm to connect to, 'xarm7' (270 platform) or 'xarm6' (1318 platform)
    """
    arm_classes = {'xarm6': xArm6, 'xarm7': xArm7}
    if arm not in arm_classes:
        raise ValueError(f'unknown arm {arm!r}; available: {sorted(arm_classes)}')
    db = Database(exp_name=exp_name) if connect_db else None
    return RobotTest_VLA(
        exp_name=exp_name,
        arm=arm_classes[arm](),
        db=db,
        to_log=to_log,
    )
