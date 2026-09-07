from sqlalchemy import Integer, String, Float, TIMESTAMP
from sqlalchemy.sql import func

from robotic_testing.common import data_analysis

configs = {

    # ec lab settings
    'protocol': {
        '1': 'OCV',
        '2': 'CV',
        '3': 'LSV',
    },

    # sql table settings
    'tables_dict': {
        'active_learning.recipe': {
            'schema': 'active_learning',
            'table_name': 'recipe',
            'columns_dict': {
                'arm_name': {'type': String(50), 'primary_key': True},
                'Ir': {'type': Float},
                'Ru': {'type': Float},
                'Ni': {'type': Float},
                'comments': {'type': String(50)},
                'created_at': {'type': TIMESTAMP, 'server_default': func.now()},
                'updated_at': {'type': TIMESTAMP, 'server_default': func.now(), 'onupdate': func.now()}
            }
        },

        'active_learning.sample': {
            'schema': 'active_learning',
            'table_name': 'sample',
            'columns_dict': {
                'id': {'type': Integer, 'primary_key': True, 'autoincrement': True},
                'trial_index': {'type': Integer},
                'sample_batch_id': {'type_': Integer},
                'arm_name': {'type': String(50)},
                'comments': {'type': String(50)},
                'created_at': {'type': TIMESTAMP, 'server_default': func.now()},
                'updated_at': {'type': TIMESTAMP, 'server_default': func.now(), 'onupdate': func.now()}
            }
        },

        'active_learning.performance': {
            'schema': 'active_learning',
            'table_name': 'performance',
            'columns_dict': {
                'id': {'type': Integer, 'primary_key': True, 'autoincrement': True},
                'sample_id': {'type': Integer},
                'total_energy': {'type': Float},
                'comments': {'type': String(50)},
                'abandoned': {'type': Integer},
                'abandon_reason': {'type': String(50)},
                'outlier': {'type': Integer},
                'created_at': {'type': TIMESTAMP, 'server_default': func.now()},
                'updated_at': {'type': TIMESTAMP, 'server_default': func.now(), 'onupdate': func.now()}
            }
        }
    },

    # data analysis settings
    'data_analysis': {
        'analyzer': {
            'analyzer_class': data_analysis.AcidicOERAnalyzer,
            'kwargs': {
                'iR_correction': True,
                'Rs': 1,
            },
        },
        'metrics': {
            'overpotential': {
                'technique_id': '3',
                'analyze_method': data_analysis.AcidicOERAnalyzer.get_overpotential
            },
            'max_i': {
                'technique_id': '3',
                'analyze_method': data_analysis.AcidicOERAnalyzer.get_max_i
            },
            'tafel_slope': {
                'technique_id': '3',
                'analyze_method': data_analysis.AcidicOERAnalyzer.get_tafel_slope,
                'kwargs': {'range': 'potential', 'p_low': 1.244, 'p_high': 1.444}
            },
            'ts_fit': {
                'technique_id': '3',
                'analyze_method': data_analysis.AcidicOERAnalyzer.get_tafel_slope_fit,
                'kwargs': {'range': 'potential', 'p_low': 1.244, 'p_high': 1.444}
            },
        }
    }

}
