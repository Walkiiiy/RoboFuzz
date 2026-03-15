import enum
import subprocess as sp
import os
from enum import Enum, auto

class TestMode(Enum):
    GENERIC = auto()
    SROS2 = auto()
    PX4_SITL = auto()
    PX4_MISSION = auto()


class RuntimeConfig:

    def __init__(self):
        config_path = os.path.abspath(__file__)
        self.src_dir = os.path.dirname(config_path)
        self.proj_root = os.path.dirname(self.src_dir)
        self.gcov_dir = os.path.join(self.src_dir, "coverage")

        self.replay = False

        self.method = None
        self.fuzz_mode = None
        self.log_dir = None
        self.queue_dir = None
        self.error_dir = None
        self.cov_dir = None
        self.meta_dir = None
        self.rosbag_dir = None
        self.maxloop = None
        self.interval = 1.0
        self.repeat = 1
        self.schedule = None
        self.seqlen = 1
        self.seed = None
        self.debug_wait = False
        self.no_cov = False

        self.persistent = False
        self.rospkg = None
        self.rosnode = None
        self.exec_cmd = None
        self.watchlist = None
        self.target_node = None
        self.target_name = None
        self.target_config = None
        self.target_warmup_sec = 0.0

        self.px4_sitl = False
        self.px4_ros = False
        self.use_mavlink = False
        self.exp_pgfuzz = False
        self.flight_mode = None
        self.px4_mission_file = None
        self.tb3_sitl = False
        self.tb3_hitl = False
        self.tb3_uri = None
        self.sros2 = False
        self.sros2_keystore = None
        self.sros2_enable = None
        self.sros2_strategy = None
        self.sros2_enclave = None
        self.test_rcl = False
        self.rcl_api = None
        self.test_rcl_feature = []
        self.test_rcl_job = None
        self.test_rcl_targets = []
        self.test_cli = False
        self.test_moveit = False
        self.test_rosidl = False
        self.test_rosidl_lang = None
        self.test_rosidl_shmid = None
        self.nav2_amcl = False
        self.post_pub_sleep = 0.0
        self.nav2_scan_len = 360
        self.nav2_map_default_width = 384
        self.nav2_map_default_height = 384
        self.fuzz_seed = None

        self.pkg_prefix = None
        self.pkg_src_dir = None
        self.ros_prefix = None
        self.pkg_cov_dir = None
        self.node_executable = None

    def find_package_metadata(self):
        # read from provided meta file
        # or find from ros installation

        ros_pkg_cmd = "ros2 pkg prefix {}".format(self.rospkg)

        if self.rospkg is None or self.rosnode is None:
            print("[warning] ros_pkg or ros_node not provided.")
            print("          Using exec_cmd instead.")
            return -1

        proc = sp.Popen(ros_pkg_cmd.split(" "), stdout=sp.PIPE)
        out = proc.stdout.read().strip()
        if len(out) == 0:
            return -1
        self.pkg_prefix = str(out, "utf-8")

        try:
            pkg_xml = os.readlink(os.path.join(self.pkg_prefix, "share",
                self.rospkg, "package.xml"))
        except OSError:
            # if not built with --symlink-install
            pkg_xml = os.path.join(self.pkg_prefix, "share", self.rospkg,
                    "package.xml")
        self.pkg_src_dir = os.path.join(os.path.dirname(pkg_xml), "src")

        self.ros_prefix = os.path.dirname(os.path.dirname(self.pkg_prefix))
        self.pkg_cov_dir = os.path.join(self.ros_prefix, "build", self.rospkg,
                "CMakeFiles", "{}.dir".format(self.rosnode), "src")

        self.node_executable = os.path.join(self.pkg_prefix, "lib",
                self.rospkg, self.rosnode)

        return 0
