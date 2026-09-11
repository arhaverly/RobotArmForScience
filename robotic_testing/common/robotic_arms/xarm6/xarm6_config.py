from robotic_testing.common.robotic_arms import pos_cal_utils

port = '192.168.1.227'

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
    'megnan_0.5': pos_cal_utils.speed_changer(pos_settings_default, ['speed', 'acc'], 0.5),
    'megnan_1': pos_cal_utils.speed_changer(pos_settings_default, ['speed', 'acc'], 1),
}

ang_settings_dict = {
    'default': ang_settings_default,
}


# pos and ang list, all y value are relative to the bottom right starting point
pos_dict = {
    'sample_starting': [603.0, 597.0, -88.0, 180.0, 0.0, 0.0],
    'warm_up': [380, None, 200, 180.0, 0.0, 0.0],
    'mid_station': [500, None, 400, 180.0, -90.0, 0.0],
    'vertical_rotation': [0, 0, 300, 0, -90, 0],
    'vertical_rotation_reverse': [0, 0, -100, 0, 90, 0],
    'flask_contact_immersed': [553, 786, 129.0, 180.0, -90.0, 0.0],
    'flask_contact_not_immersed': [562, 786, 145.0, 180.0, -90.0, 0.0],
    'flask_without_cap': [562, 786, 145.0, 180.0, -90.0, 0.0],
    'rinsing': [415.0, 975.0, 20.0, 180.0, -90.0, 43.0],
    'rinsing_shake': [10, 10, 10, 0, 0, 0],
    'backward_avoid_hooking': [-2, 0, 0, 0, 0, 0],
}

ang_dict = {
    'home': [90.0, -30.0, 0.0, 0.0, 30.0, 0.0],
    'stretch': [0.0, -30.0, -30.0, 0.0, 0.0, 0.0],
}

hover_offset_dict = {
    'sample': 100,
    'flask': 150,
    'rinsing': 250
}

linear_track_dict = {
    'rinsing': {'lin_pos_high_lim': 450}
}

# position calculator
sample_pos_config = {
    'grid_d': 25,
    'num_slot': 6,
    'rack_grid_pos': {
        0: (0, 0),
        1: (0, -5),
        2: (0, -10),
        3: (0, -15),
        4: (0, -20),
        5: (-5, -0),
        6: (-5, -5),
        7: (-5, -10),
        8: (-5, -15),
        9: (-5, -20),
    },
    'rack_offset': {
        0: [(0, 0, 0), (0, 0, 0)],
        1: [(0, 0, 0), (0, 0, 0)],
        2: [(0, 0, 0), (0, 0, 0)],
        3: [(0, 0, 0), (0, 0, 0)],
        4: [(0, 0, 0), (0, 0, 0)],
        5: [(0, -4, 0), (0, -4, 0)],
        6: [(0, -4, 0), (0, -4, 0)],
        7: [(0, -4, 0), (0, -4, 0)],
        8: [(0, -4, 0), (0, -4, 0)],
        9: [(0, -4, 0), (0, -4, 0)],
    },
    'slot_layout_on_rack': [0, -20, -40, -60, -80, -100],
    'rack_grid_starting_pos': pos_dict['sample_starting'],
    'contact_foil_offset': 0.7
}

pos_cal = pos_cal_utils.SamplePosCal(**sample_pos_config)


# other variables
gripper_open_dist = 55


# ----------------------------------------------------------------------
# Safety envelope for free-form / VLA-commanded motion
# ----------------------------------------------------------------------
# IMPORTANT: unlike pos_dict, these ranges are in the arm's OWN base frame -- the
# coordinates get_position() reports -- not the lab frame used by pos_dict. On this
# platform the y values in pos_dict are lab coordinates that move_to_pos() splits into a
# linear-track position plus an arm-frame residual, so the arm itself only ever sees the
# residual. move_relative() drives the arm only and never the linear track.
#
# Arm-frame poses this platform is known to reach safely:
#   sample grid   x 378 .. 603,  y 0    (track carries the lab y),  z -88 (+100 hover -> 12)
#   warm_up       x 380,         y unchanged,                       z 200
#   mid_station   x 500,         y unchanged,                       z 400
#   flask         x 553,         y 86   (track at its 700 limit),   z 129 (+150 hover -> 279)
#   rinsing       x 415,         y 525  (track at its 450 limit),   z 20  (+250 hover -> 270)
#
# Before widening any range, physically check that the volume you are opening up is clear,
# and remember the whole envelope travels with the linear track.
#
# WARNING: these ranges are station-derived and have NOT been measured on this arm.
# The same mistake found on the xArm7 almost certainly applies here: `home` is defined
# in joint space (ang_dict['home'] starts with J1 = 90 deg, so the arm is rotated a
# quarter turn away from the stations) and its TCP pose is nowhere in this file, so
# x_range below probably excludes it. Run robotic_testing/measure_envelope.py --arm xarm6
# and replace these numbers before driving this arm freely.
safety_dict = {
    'x_range': (330.0, 660.0),
    'y_range': (-120.0, 560.0),
    'z_range': (-100.0, 500.0),

    'max_reach_mm': 700.0,
    'min_reach_mm': 200.0,

    'max_step_mm': 100.0,
    'max_step_deg': 30.0,
    'max_plan_actions': 40,
    'max_wait_s': 60.0,

    'settings_profile': 'slow_1',

    # both of these leave y as None, so they move the arm without touching the linear track
    'named_positions': ('mid_station', 'warm_up'),

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
        'joint': 6,
        'joint_limit_deg': 170.0,
        'settings_profile': 'default',
    },
}

# gripper opening below which the arm assumes it is holding the sample holder
sample_holder_gripper_threshold = 20
