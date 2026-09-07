from robotic_testing import data_analysis

configs = {
    # file path settings
    'data_dir': 'D:/Zhen/DFFC_PdPtCu',

    # ec lab settings
    'protocol': {
        '1': 'OCV',
        '2': 'CV',
        '3': 'LSV',
    },

    # data analysis settings
    'data_analysis': {
        'analyzer': {
            'analyzer_class': data_analysis.AlklineFORAnalyzer,
            'kwargs': {},
            },
        'metrics': {
            'max_power': {
                'technique_id': '3',
                'analyze_method': data_analysis.AlklineFORAnalyzer.get_max_power,
                'kwargs': {'ref_potential': -0.924, 'counter_potential': 1}
            },
        },
    },

}
