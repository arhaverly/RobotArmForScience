from db_control.database import Database
import pandas as pd
import os
from matplotlib.lines import Line2D
import matplotlib.pyplot as plt
import matplotlib.transforms as transforms
from utils.utils import config_loader


class Plotter:
    def __init__(self, exp_name, test_data_ranges: list, plot_batch_name):
        self.exp_name = exp_name
        self.db = Database(exp_name, to_log=False)
        self.configs = config_loader(exp_name, 'robot_test_configs')
        self.data = self.load_data(test_data_ranges)
        self.col_value_ordered_list = self.get_col_value_ordered_list(self.data)
        self.sample_data_dict = self.group_data_by_sample()
        self.save_dir = self.get_plots_save_dir(plot_batch_name)

        # plot utils
        self.markers = {0: 'o', 1: '^', 2: 's', 3: '*', 4: 'D', 5: 'X'}
        self.colors = [f'C{i}' for i in range(10)]

    def load_data(self, test_data_ranges: list):
        metric_list = list(self.configs['data_analysis']['metrics'].keys())
        metric_str = ', '.join(metric_list)
        data_chunks = [pd.read_sql(
                f'SELECT arm_name, sample_id, sample_batch_id, test_id, {metric_str} '
                f'FROM active_learning.full_table WHERE test_id BETWEEN {test_data_range[0]} AND '
                f'{test_data_range[1]} AND abandoned IS NULL',
                self.db.engine.connect()
            )
            for test_data_range in test_data_ranges]
        return pd.concat(data_chunks, ignore_index=True)


    @staticmethod
    def get_col_value_ordered_list(data_df):
        """
        an ordered list of all values under a col with no duplicates
        """
        col_value_ordered_list = {}
        for col_name in data_df.columns:
            col_value_ordered_list[col_name] = sorted(list(set(data_df[col_name])))
        return col_value_ordered_list

    def group_data_by_sample(self):
        sample_data_dict = {}
        for sample_id in self.col_value_ordered_list['sample_id']:
            sample_data_dict[sample_id] = self.data[self.data['sample_id'] == sample_id]
        return sample_data_dict

    @staticmethod
    def get_plots_save_dir(plot_batch_name):
        save_dir = f'{plot_batch_name}'
        os.makedirs(save_dir, exist_ok=True)
        return save_dir

    @staticmethod
    def plot_default_settings(ax, fig):
        ax.set_axisbelow(True)
        ax.tick_params(axis='both', labelsize=13)

        # axis x settings
        ax.set_xlabel('arm_name')
        ax.xaxis.label.set_size(15)

        # axis y settings
        ax.set_ylabel('max_power (mW)')
        ax.yaxis.grid(True, color='#EEEEEE')
        ax.yaxis.label.set_size(15)

        # background color
        fig.set_facecolor('white')

    @staticmethod
    def make_legend(ax, legend_elements, loc):
        legend = ax.legend(handles=legend_elements, loc=loc, fontsize=13, fancybox=True, framealpha=0.2)
        ax.add_artist(legend)

    def make_sample_batch_id_legend(self, ax):
        sample_batch_id_list = self.col_value_ordered_list['sample_batch_id']
        legend_elements = [
            Line2D([0], [0], marker='o', color=self.colors[i], linestyle='None', label=f'sample_batch_{sample_batch_id_list[i]}')
            for i in range(len(sample_batch_id_list))
        ]
        self.make_legend(ax, legend_elements, 'upper left')

    def make_sample_duplicate_legend(self, ax):
        max_samples = self.data.groupby(['arm_name', 'sample_batch_id'])['sample_id'].unique().apply(len).max()
        legend_elements = [
            Line2D([0], [0], marker=self.markers[i], color='grey', linestyle='None', label=f'sample_duplicate_{i + 1}')
            for i in range(max_samples)
        ]
        self.make_legend(ax, legend_elements, 'upper right')

    def get_sample_duplicate_ordered_list(self, sample_batch_id, arm_name):
        """
        the list of sample_id that has the same sample_batch_id and arm_name
        """
        sample_id_list = self.col_value_ordered_list['sample_id']
        sample_duplicate_ordered_list = []
        for sample_id in sample_id_list:
            sample_data = self.sample_data_dict[sample_id]
            if sample_data['sample_batch_id'].iloc[0] == sample_batch_id and sample_data['arm_name'].iloc[0] == arm_name:
                sample_duplicate_ordered_list.append(sample_id)
        return sorted(sample_duplicate_ordered_list)

    def get_fig_ax(self):
        width = max(10.0, 1.5 * len(self.col_value_ordered_list['arm_name']))
        figsize = (width, 8)
        return plt.subplots(figsize=figsize)

    def plot(self, title):
        fig, ax = self.get_fig_ax()
        self.plot_default_settings(ax, fig)

        # set x shift
        shift_step = 1 / 6
        offset = lambda x: transforms.ScaledTranslation(shift_step * x, 0, plt.gcf().dpi_scale_trans)
        trans = ax.transData

        for sample_id, sample_data in self.sample_data_dict.items():

            points_data = []

            # get color by sample_batch_id
            sample_batch_id = sample_data['sample_batch_id'].iloc[0]
            color_id = self.col_value_ordered_list['sample_batch_id'].index(sample_batch_id)

            # get marker by sample_id
            sample_duplicate_ordered_list = self.get_sample_duplicate_ordered_list(
                sample_batch_id, sample_data['arm_name'].iloc[0]
            )
            marker_id = sample_duplicate_ordered_list.index(sample_id)

            for i, sample_test_data in sample_data.sort_values('test_id').iterrows():

                # get shift by test_id
                test_id_ordered_list = self.get_col_value_ordered_list(sample_data)['test_id']
                shift_id = test_id_ordered_list.index(sample_test_data['test_id'])

                # plot points
                point = ax.scatter(
                    sample_test_data['arm_name'],
                    sample_test_data['max_power'],
                    c=self.colors[color_id],
                    marker=self.markers[marker_id],
                    alpha=0.7,
                    s=45,
                    transform=trans + offset(shift_id) if shift_id else trans
                )

                points_data.append(point.get_offsets()[0])

            # plot lines
            for i in range(len(points_data) - 1):
                x1, y1 = points_data[i]
                x2, y2 = points_data[i + 1]
                ax.plot([x1 + shift_step * i, x2 + shift_step * (i + 1)], [y1, y2],
                        linestyle='dashed', color=self.colors[color_id], alpha=0.5, linewidth=2)

        self.make_sample_batch_id_legend(ax)
        self.make_sample_duplicate_legend(ax)

        # title
        plot_title = f'{title}'
        ax.set_title(plot_title, fontsize=20, pad=20)

        # saving
        plt.savefig(f'{self.save_dir}/{plot_title}.png', dpi=300, bbox_inches='tight')
        plt.close()


