import ctypes
import importlib
import logging
import os

import moviepy.video.fx.all as vfx
import numpy as np
import pandas as pd
import pyautogui as pya
import requests
import yagmail
from moviepy.editor import VideoFileClip, AudioFileClip
from pydub import AudioSegment

from utils.sensitives import ip_ot2, ip_rt, crest_gmail, crest_app_pw


def get_project_path():
    path = os.getcwd()
    while os.path.basename(path) != 'catalyst':
        path = os.path.dirname(path)
    # if in win, replace \ with /
    path = path.replace('\\', '/')
    return path


def get_logger(exp_name, module_name):
    logger = logging.getLogger(module_name)
    logger.setLevel(logging.INFO)
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    # create log file if not exist
    log_dir = get_dir('log_dir', exp_name=exp_name)
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
    file_handler = logging.FileHandler(get_log_path(exp_name))
    file_handler.setLevel(logging.INFO)
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)
    return logger


def complement_value(parameters: dict):
    sum_value = np.sum(list(parameters.values()))
    comp = 1 - sum_value
    # set to 0 if it's too small or even a negative value
    comp = comp if comp > 0.002 else 0
    return round(comp, 3)


def turn_capslock_off():
    if ctypes.WinDLL("User32.dll").GetKeyState(0x14):
        pya.press("capslock")


def log_and_print(to_log: bool, logger: logging.Logger, log_level: str, msg: str):
    if to_log:
        level_dict = {'debug': logger.debug, 'info': logger.info, 'warning': logger.warning, 'error': logger.error}
        level_dict[log_level](msg)
    if log_level in ['warning', 'error']:
        print(msg)


def config_loader(exp_name, config_type):
    assert config_type in [
        'ot2_deck_configs',
        'ot2_run_configs',
        'database_configs',
        'robot_test_configs',
        'al_configs'
    ]

    # load default config
    default_config = importlib.import_module(f'projects.default.config_files.{config_type}').configs

    # load project config
    try:
        project_config = importlib.import_module(f'projects.{exp_name}.config_files.{config_type}').configs
    except ModuleNotFoundError:
        project_config = {}

    # update default config with project config
    config = {**default_config, **project_config}

    return config


def get_tasks(exp_name, task_name):
    tasks = importlib.import_module(f'projects.{exp_name}.config_files.ot2_tasks_configs').configs[task_name]
    task_df = pd.DataFrame(columns=tasks['elements'], data=tasks['task_list'])
    print(f'{task_df}\n\n'
          f'Please double check task list as above')
    return task_df


def get_dir(dir_name, exp_name=None):
    assert dir_name in ['log_dir', 'icons_dir']
    if dir_name == 'log_dir':
        return os.path.expanduser(f"~/PycharmProjects/catalyst/projects/{exp_name}/logs")
    elif dir_name == 'icons_dir':
        return os.path.expanduser(f"~/PycharmProjects/catalyst/robotic_testing/icons")


def get_log_path(exp_name):
    return os.path.expanduser(f"~/PycharmProjects/catalyst/projects/{exp_name}/logs/{exp_name}.log")


def normalize_task(task: pd.DataFrame):
    task_norm = task.copy()
    for task_index, task_recipe in task_norm.iterrows():
        # skip if all 0
        if any(task_recipe) != 0:
            task_norm.iloc[task_index] = np.round(task_norm.iloc[task_index] / task_norm.iloc[task_index].sum(), 3)
    return task_norm


def get_latest_file_name(dir_path):
    """
    Get the last modified file name only in a directory
    """
    files = os.listdir(dir_path)
    paths = [os.path.join(dir_path, basename) for basename in files]
    full_path = max(paths, key=os.path.getmtime)
    return os.path.splitext(os.path.basename(full_path))[0]


def hyperlapse_video(file_name, input_path, output_path, input_type='mkv', output_type='mp4', acceleration_factor=100,
                     audio=False):
    """
    Hyperlapse a video to speed up the video by acceleration_factor
    """
    input_file = f'{os.path.join(input_path, file_name)}.{input_type}'
    output_file = f'{os.path.join(output_path, file_name)}_{acceleration_factor}x.{output_type}'

    clip = VideoFileClip(input_file)
    audio_clip = None

    if audio:
        audio_clip = clip.audio
        audio_clip.write_audiofile("temp_audio.wav")
        sound = AudioSegment.from_file("temp_audio.wav")
        sound = sound.speedup(playback_speed=acceleration_factor)
        sound.export("temp_audio_speedup.wav", format="wav")
        audio_clip = AudioFileClip("temp_audio_speedup.wav")
    else:
        clip = clip.without_audio()

    clip = clip.set_fps(30)

    final = clip.fx(vfx.speedx, acceleration_factor)

    if audio:
        final = final.set_audio(audio_clip)

    final.write_videofile(output_file, codec='h264')

    return output_file


def email(email_address, subject, contents: list or str):
    yag = yagmail.SMTP(crest_gmail, crest_app_pw)
    yag.send(email_address, subject, contents)


def post_to_ot2(endpoint, data):
    response = requests.post(f'{ip_ot2}/{endpoint}', json=data)
    return response


def post_to_rt(endpoint, data):
    response = requests.post(f'{ip_rt}/{endpoint}', json=data)
    return response


def print_gpt_process(content, gpt_process):
    assert gpt_process in ['msg_received', 'msg_replied', 'func_called', 'func_completed']
    mapping = {
        'msg_received': 'MESSAGE RECEIVED',
        'msg_replied': 'MESSAGE REPLIED',
        'func_called': 'FUNCTION CALLED',
        'func_completed': 'FUNCTION COMPLETED',
    }
    print(f"\n****{mapping[gpt_process]}****\n\n"
          f"{content}\n\n"
          f"-----------------------\n")


def get_counterpart(var):
    counterparts = {'start': 'stop', 'stop': 'start'}
    return counterparts.get(var, 'Invalid input')
