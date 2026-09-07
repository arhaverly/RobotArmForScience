configs = {
    'layout': {
        #'1': 'piece_sample_stage_steel',
        '1': 'strip_sample_stage',
        '2': 'nest_96_wellplate_200ul_flat',
        '3': 'opentrons_15_tuberack_falcon_15ml_conical',
        '4': 'piece_sample_stage_wood',
        '5': 'nest_96_wellplate_200ul_flat',
        '6': 'opentrons_96_tiprack_300ul',
        '7': 'opentrons_96_tiprack_20ul',
        '8': 'opentrons_96_tiprack_20ul',
        '9': 'opentrons_96_tiprack_300ul',
        '10': 'opentrons_96_tiprack_20ul',
        '11': 'opentrons_96_tiprack_20ul',
    },

    # pipette rate settings
    'pipetting_rate_to_mixing_well': 1,
    'pipetting_rate_to_sample': 0.05,

    # mixing settings
    'mixing_tip_type': '300ul',
    'mixing_vol': 300,
    'mixing_times': 10,
    'mixing_rate': 3,

    # total vol calculation settings, if mixing vol too small, may lead to mixing failure
    'vol_offset_multiplier': 2.8, #1.8 default
    'vol_offset_addition': 60, #30

    # reservoir location records the name and the layout of each reservoir
    'reservoirs_layout': {
        'reservoir_1': {
            'Pt': 'A1',
            'Ru': 'A2',
            'Ir': 'A3',
            'Sn': 'A4',
            'Au': 'A5',
            'Cr': 'B1',
            'Fe': 'B2',
            'Co': 'B3',
            'Ni': 'B4',
            'Cu': 'B5',
            #'Hf': 'C2',
            'In': 'C2',
            #'Nb':'C3',
            'Ce': 'C3',
            'W':  'C4',
            'Ca': 'C5',
        },
    },
}
