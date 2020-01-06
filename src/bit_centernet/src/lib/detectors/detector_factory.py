from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import _init_paths
import os
import sys
sys.path.append(os.getcwd())
from .exdet import ExdetDetector
from .ddd import DddDetector
#L_Detector
from .ctdet import CtdetDetector
from .multi_pose import MultiPoseDetector
from .kps_detector import KPSDetector

detector_factory = {
  'exdet': ExdetDetector, 
  'ddd': DddDetector,
  'ctdet': CtdetDetector,
  'multi_pose': MultiPoseDetector,
  'KPS':KPSDetector
}
