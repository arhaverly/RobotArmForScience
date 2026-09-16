from robotic_testing.common.robotic_arms import pos_cal_utils

# port = '192.168.1.228'
port = '192.168.1.209' # new robot

# robotic arm movement parameter settings
pos_settings_default = {
    'speed': 100,
    'acc': 2000,
    'radius': -1,
    'events': {},
    'callback_in_thread': True,
    'quit': False,
    'wait': True,
}

ang_settings_default = {
    'angle_speed': 20,
    'angle_acc': 500,
    'radius': -1,
    'wait': True,
}

pos_settings_dict = {
    'default': pos_settings_default,
    'slow_1': pos_cal_utils.speed_changer(pos_settings_default, ['speed', 'acc'], 0.5),
    'slow_2': pos_cal_utils.speed_changer(pos_settings_default, ['speed', 'acc'], 0.25),
    'slow_3': pos_cal_utils.speed_changer(pos_settings_default, ['speed', 'acc'], 0.125),
    'fast_1': pos_cal_utils.speed_changer(pos_settings_default, ['speed', 'acc'], 2),
}

ang_settings_dict = {
    'default': ang_settings_default,
}


# pos and ang list
# Previous version
# pos_dict = {
#     'sample_starting': [-409.0, 150.0, 190.0, 180.0, 0.0, 180.0],
#     'mid_station': [-309, 350, 250, 180.0, 0.0, 180.0],
#     'vertical_rotation': [0, 0, 100, 0, -90, 0],
#     'vertical_rotation_reverse': [0, 0, 0, 0, 90, 0],
#     'flask_contact_immersed': [-440.0, -48, 252.0, 180.0, -90.0, 180.0],
#     '7cm_sample_XAS': [-440.0, -48, 312.0, 180.0, -90.0, 180.0],
#     '6cm_sample_XAS': [-440.0, -48, 302.0, 180.0, -90.0, 180.0],
#     'flask_contact_not_immersed': [-440.0, -50, 257.0, 180.0, -90.0, 180.0],
#     'flask_without_cap': [-440.0, -50, 240.0, 180.0, -90.0, 180.0],
#     'small_cell': [-440.0, -48, 242.0, 180.0, -90.0, 180.0],
#     'rinsing': [-488.0, 208.0, 140.0, 180.0, -90.0, 180.0],
#     'rinsing_shake': [-488.0, 208.0, 148.0, 180.0, -90.0, 180.0],
#     'backward_avoid_hooking': [2, 0, 0, 0, 0, 0]
#     # [x, y, z, ]
# }
#
# ang_dict = {
#     'home': [60.0, 0.0, 0.0, 0.0, 0.0, -60.0, 0.0],
#     'stretch': [60.0, 0.0, 0.0, 60.0, 0.0, -30.0, 0.0],
# }

# 2026/01/22 new robot
pos_dict = {
    'sample_starting': [153, 408.1, 190.6, 180.0, 0.0, 90.0],  # NEW
    'mid_station': [347.9, 308.1, 251.5, 179.7, 0.1, 90.1], # NEW
    'vertical_rotation': [0, 0, 100, 0, -90, 0],
    'vertical_rotation_reverse': [0, 0, 0, 0, 90, 0],
    'flask_contact_immersed': [-51.5, 257.4, 245, 87, -90, -178],
    # '7cm_sample_XAS': [-440.0, -48, 312.0, 180.0, -90.0, 180.0],
    # '6cm_sample_XAS': [-440.0, -48, 302.0, 180.0, -90.0, 180.0],
    # 'flask_contact_not_immersed': [-440.0, -50, 257.0, 180.0, -90.0, 180.0],
    # 'flask_without_cap': [-440.0, -50, 240.0, 180.0, -90.0, 180.0],
    # 'small_cell': [-440.0, -48, 242.0, 180.0, -90.0, 180.0],
    'rinsing': [195.8, 322.6, 140.0, 87.0, -90, -178],
    'rinsing_shake': [195.8, 322.6, 148.0, 87.0, -90, -178],
    'backward_avoid_hooking': [0, -2, 0, 0, 0, 0]
    # [x, y, z, ]
}

# WARNING: these are joint angles, and NOTHING CHECKS THEM.
#
# home() commands them through set_servo_angle, which the safety envelope below does not
# touch -- vla_control's executor says so outright ("joint-space move, so the resulting
# TCP pose is not predicted here") and returns no pose to check. So the envelope has
# never guarded home(), and widening or narrowing it cannot make home() safer.
#
# Whether these angles put the gripper above the bench or into it is a fact about how
# this arm is mounted, and it is recorded nowhere. They were last changed for the robot
# swap in "using 1326 robot instead" and have not been verified against this bench since.
# On 2026-09-16 home() drove the arm into the table.
#
# Before trusting home() again: jog the arm to a safe resting pose by hand or in
# UFACTORY Studio, read the joint angles off the controller, and put those here. Do not
# guess them, and note that home() goes through 'stretch' first, so that pose has to be
# clear of the bench too -- both the poses and the swing between them.
ang_dict = {
    'home': [0.0, 0.0, 0.0, 0.0, 0.0, -60.0, 0.0],
    'stretch': [0.0, 0.0, 0.0, 30.0, 0.0, -30.0, 0.0],
}


hover_offset_dict = {
    'sample': 100,
    'flask': 156,
    'rinsing': 150
}


# position calculator
sample_pos_config = {
    'grid_d': 50,
    'num_slot': 6,
    'rack_grid_pos': {
        0: (0, 0),
        1: (3, 0),
        2: (6, 0),
        3: (0, -3),
        4: (3, -3),
        5: (6, -3),
    },
    'rack_offset': {
        0: [(0, 1.7, 0), (0, 1.4, 0)],
        1: [(0, 0.5, 2), (1, 0.4, 3)],
        2: [(-2, 1.5, 4), (-3.5, 0.5, 7)],
        3: [(0.5, 0, -4), (0.5, 0, -3)],
        4: [(0.5, 1, -2), (0, 0.7, -1)],
        5: [(0, 0.3, 0.5), (0, 0.3, 2)],
    },
    'slot_layout_on_rack': [0, 20, 40, 60, 80, 100, 150],
    'rack_grid_starting_pos': pos_dict['sample_starting'],
    'contact_foil_offset': 0.7
}

pos_cal = pos_cal_utils.SamplePosCal(**sample_pos_config)


# other variables
gripper_open_dist = 55


# ----------------------------------------------------------------------
# Safety envelope for free-form / VLA-commanded motion
# ----------------------------------------------------------------------
# These limits are what xArm.move_relative() and friends are held to.
#
# The station poses below come from pos_dict:
#
#   sample grid   x 152 .. 553,  y 258 .. 410,  z 191  (+100 hover -> z 291)
#   mid_station   x 348,         y 308,         z 252
#   flask         x -52,         y 257,         z 245  (+156 hover -> z 401)
#   rinsing       x 196,         y 323,         z 140  (+150 hover -> z 290)
#
# `home` is defined in joint space (ang_dict), so its TCP pose appears nowhere in this
# file: measured on the real arm it sits at y ~= 2, outside the band the stations occupy.
#
# y_range was once widened to -80 to cover that, on the reasoning that the station
# routines sweep the TCP across the whole band anyway. That was wrong twice over, and it
# is reverted here:
#
#   - What a joint-space routine sweeps through is not what a straight Cartesian line
#     between two points in a box passes through. Declaring the bounding box of a
#     trusted path safe hands out the whole volume, including the part over the bench.
#   - It treated "home is outside the envelope" as a fault in the envelope. Whether
#     home is somewhere the arm should be is a question about this bench, answered by
#     measuring, not by widening a limit until the complaint stops.
#
# So y_range is back to the station-derived band. The consequence is deliberate: a
# free-form move commanded from home is refused, because the envelope does not vouch
# for where home is. Refusing to move is the safe answer to not knowing.
#
# Use robotic_testing/measure_envelope.py to derive these numbers from poses the arm
# actually reaches. It only reads the arm and commands no motion. Anything inside the
# box that the arm must not hit means pulling a range back, not widening it; the z floor
# is the one that keeps it off the bench.
#
# TODO: x_range and z_range are still station-derived, and none of this constrains a
# joint-space move -- see the note on ang_dict above.
safety_dict = {
    'x_range': (-130.0, 600.0),
    'y_range': (180.0, 480.0),
    'z_range': (140.0, 560.0),

    # keep the wrist out of the far edge of the working radius and away from the base column
    'max_reach_mm': 700.0,
    'min_reach_mm': 180.0,

    # a planner has to express a long move as several short ones, which keeps every
    # individual command small enough to interrupt and easy to reason about
    'max_step_mm': 100.0,
    'max_step_deg': 30.0,
    'max_plan_actions': 40,
    'max_wait_s': 60.0,

    # free-form motion runs at half the normal speed
    'settings_profile': 'slow_1',

    # pos_dict keys a planner may drive to directly; station poses such as the immersed
    # flask depth are deliberately left out
    'named_positions': ('mid_station',),

    'gripper': {
        'object_grip_opening': 0,
        'object_detect_min': 3.0,
    },
    'pick_place': {
        'descend': 60.0,
        'lift': None,
        'settings_profile': 'slow_2',
    },
    'wave': {
        'times': 3,
        'amplitude_deg': 25.0,
        'joint': 7,
        'joint_limit_deg': 170.0,
        'settings_profile': 'default',
    },
}

# gripper opening below which the arm assumes it is holding the sample holder
sample_holder_gripper_threshold = 20
