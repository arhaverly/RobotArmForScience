configs = {

    'layout': {
        # axygen_1_reservoir_90ml is a surrogate for sample stage
        '1': 'axygen_1_reservoir_90ml',
        '2': 'axygen_1_reservoir_90ml',
        '3': 'opentrons_15_tuberack_falcon_15ml_conical',
        '4': 'nest_96_wellplate_200ul_flat',
        '5': 'nest_96_wellplate_200ul_flat',
        '6': 'opentrons_15_tuberack_falcon_15ml_conical',
        '7': 'opentrons_96_tiprack_20ul',
        '8': 'opentrons_96_tiprack_20ul',
        '9': 'opentrons_96_tiprack_300ul',
        '10': 'Temperature Module',
        '11': 'opentrons_96_tiprack_300ul',
    },

    'reservoirs_loc': {
        'reservoir_1': '3',
        'reservoir_2': '6'
    },

    'reservoirs_layout': {
        'reservoir_1': {
            'Pd': 'A1',
            'Pt': 'A2',
            'Cu': 'A3',
        },

        'reservoir_2': {

        },
    },

    'sample_stage_grid_offset': [43.76, 75.8, 10.23],
}

