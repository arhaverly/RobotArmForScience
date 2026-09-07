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
