from dataclasses_json import dataclass_json
from dataclasses import dataclass
from typing import List, Union, Optional
from enum import IntEnum
from pathlib import Path
import json5
import os


class WaysideIndex(IntEnum):
    wayside1 = 1
    wayside2 = 2
    wayside3 = 3

class CameraIndex(IntEnum):
    camera1 = 1
    camera2 = 2
    camera3 = 3

class LidarIndex(IntEnum):
    lidar1 = 1

@dataclass_json
@dataclass
class Host:
    index: WaysideIndex
    address: str

@dataclass_json
@dataclass
class Session:
    name: str
    hosts: List[Host]

def load_session(path: Path) -> Session:
    assert os.path.isabs(path), 'Absolute path is mandantory!'
    with open(path) as f:
        return Session.from_dict(json5.load(f))  # type: ignore


@dataclass_json
@dataclass
class DeviceList:
    index: WaysideIndex
    camera_list: List[CameraIndex]
    lidar_list: List[LidarIndex]

@dataclass_json
@dataclass
class Recipe:
    name: str
    timeout_secs: int
    device_lists: List[DeviceList]
    since: Optional[str] = None

def load_recipe(path: Path) -> Recipe:
    assert os.path.isabs(path), 'Absolute path is mandantory!'
    with open(path) as f:
        return Recipe.from_dict(json5.load(f))  # type: ignore


def display(dev_list: Union[List[CameraIndex], List[LidarIndex]]) -> str:
    return ' '.join(map(str, (map(int, dev_list))))
