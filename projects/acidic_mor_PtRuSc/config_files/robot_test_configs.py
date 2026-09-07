from robotic_testing import data_analysis

configs = {
    # file path settings
    'data_dir': 'D:/Zhichu/acidic_mor_PtRuSc',

    # ec lab settings
    'protocol': {
        '1': 'OCV',
        '2': 'CV',
        '3': 'LSV',
    },

    # data analysis settings
    'data_analysis': {
        'analyzer': {
            'analyzer_class': data_analysis.AcidicMORAnalyzer,
            'kwargs': {
                'iR_correction': False,
                'Rs': 1,
            },
        },
        'metrics': {
            'max_power': {
                'technique_id': '3',
                'analyze_method': data_analysis.AcidicMORAnalyzer.get_max_power,
                'kwargs': {'ref_potential': -0.263, 'counter_potential': 0.9}
            },
        }
    }

}
