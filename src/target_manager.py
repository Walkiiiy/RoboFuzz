"""Target registry and lifecycle manager.

This module enables configuration-driven target onboarding by scanning
`<project_root>/targets/*/config.json` at runtime.
"""

from __future__ import annotations

import importlib.util
import json
import os
import signal
import subprocess as sp
import time
from typing import Dict, Optional

from target_plugins import BaseTargetPlugin


class TargetManager:
    """Load, register, and manage fuzzing targets from config files."""

    def __init__(self, proj_root: str, runtime_config=None) -> None:
        self.proj_root = proj_root
        self.runtime_config = runtime_config
        self.targets_roots = self._get_candidate_target_roots()
        self.registry: Dict[str, dict] = {}
        self.process_map: Dict[str, sp.Popen] = {}
        self._scan_targets()

    def _get_candidate_target_roots(self):
        roots = []
        env_root = os.getenv("ROBOFUZZ_TARGETS_DIR")
        if env_root:
            roots.append(env_root)

        roots.append(os.path.join(self.proj_root, "targets"))

        src_dir = getattr(self.runtime_config, "src_dir", None)
        if src_dir:
            roots.append(os.path.join(src_dir, "targets"))
            roots.append(os.path.join(os.path.dirname(src_dir), "targets"))

        roots.append(os.path.join(os.getcwd(), "targets"))
        roots.append(os.path.join(os.getcwd(), "..", "targets"))

        uniq = []
        seen = set()
        for root in roots:
            root = os.path.abspath(root)
            if root in seen:
                continue
            seen.add(root)
            uniq.append(root)
        return uniq

    def _scan_targets(self) -> None:
        for root in self.targets_roots:
            if not os.path.isdir(root):
                continue
            for entry in sorted(os.listdir(root)):
                target_dir = os.path.join(root, entry)
                config_path = os.path.join(target_dir, "config.json")
                if not os.path.isdir(target_dir) or not os.path.isfile(config_path):
                    continue

                with open(config_path, "r") as f:
                    cfg = json.load(f)

                name = cfg.get("name") or entry
                cfg["name"] = name
                cfg["target_dir"] = target_dir
                cfg["config_path"] = config_path
                self.registry[name] = cfg

    def available_targets(self):
        return sorted(self.registry.keys())

    def get_target_config(self, target_name: str) -> dict:
        if target_name not in self.registry:
            raise KeyError(
                f"target '{target_name}' not registered. "
                f"available={self.available_targets()} "
                f"searched_roots={self.targets_roots}"
            )
        return self.registry[target_name]

    def apply_target_to_runtime(self, target_name: str, runtime_config) -> dict:
        cfg = self.get_target_config(target_name)
        runtime_config.target_name = target_name
        runtime_config.target_config = cfg

        basic = cfg.get("basic", {})
        lifecycle = cfg.get("lifecycle", {})
        monitoring = cfg.get("monitoring", {})

        runtime_config.rospkg = basic.get("ros_pkg", runtime_config.rospkg)
        runtime_config.rosnode = basic.get("ros_node", runtime_config.rosnode)
        runtime_config.exec_cmd = lifecycle.get("exec_cmd", runtime_config.exec_cmd)

        watchlist_rel = monitoring.get("watchlist")
        if watchlist_rel:
            runtime_config.watchlist = self.resolve_target_path(cfg, watchlist_rel)

        warmup = lifecycle.get("warmup_sec", 0)
        runtime_config.target_warmup_sec = float(warmup)
        for key, value in cfg.get("runtime", {}).items():
            if isinstance(value, str):
                path_like_keys = {
                    "sros2_keystore",
                    "fuzz_seed",
                    "px4_mission_file",
                }
                if key.endswith("_path") or key in path_like_keys:
                    value = self.resolve_target_path(cfg, value)
            setattr(runtime_config, key, value)

        return cfg

    def resolve_target_path(self, target_cfg: dict, rel_or_abs: str) -> str:
        if os.path.isabs(rel_or_abs):
            return rel_or_abs

        resolved = os.path.normpath(
            os.path.join(target_cfg["target_dir"], rel_or_abs)
        )
        if os.path.exists(resolved):
            return resolved

        # Fallback for mixed mount layouts:
        # target configs may live under either <repo>/targets or <repo>/src/targets.
        # If the relative path cannot be resolved from target_dir, try canonical
        # locations under src/.
        base_name = os.path.basename(rel_or_abs)
        src_dir = getattr(self.runtime_config, "src_dir", None)
        candidates = []
        if src_dir:
            candidates.append(os.path.join(src_dir, "watchlist", base_name))
            candidates.append(os.path.join(src_dir, base_name))
        candidates.append(os.path.join(self.proj_root, "src", "watchlist", base_name))
        candidates.append(os.path.join(self.proj_root, "src", base_name))

        for cand in candidates:
            cand = os.path.abspath(cand)
            if os.path.exists(cand):
                return cand

        return resolved

    def _load_plugin_class(self, target_cfg: dict):
        plugin_rel = target_cfg.get("monitoring", {}).get("plugin")
        if not plugin_rel:
            return None

        plugin_path = self.resolve_target_path(target_cfg, plugin_rel)
        if not os.path.isfile(plugin_path):
            raise FileNotFoundError(f"plugin file not found: {plugin_path}")

        module_name = f"target_plugin_{target_cfg['name']}"
        spec = importlib.util.spec_from_file_location(module_name, plugin_path)
        if spec is None or spec.loader is None:
            raise ImportError(f"cannot import plugin module from {plugin_path}")

        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        plugin_cls = getattr(module, "TargetPlugin", None)
        if plugin_cls is None:
            raise AttributeError(
                f"plugin module {plugin_path} must expose class TargetPlugin"
            )
        return plugin_cls

    def get_plugin(self, target_name: str, runtime_config=None) -> BaseTargetPlugin:
        cfg = self.get_target_config(target_name)
        plugin_cls = self._load_plugin_class(cfg)
        if plugin_cls is None:
            return None
        return plugin_cls(cfg, runtime_config or self.runtime_config)

    def get_field_whitelist(self, target_name: str, msg_type_name: Optional[str] = None):
        cfg = self.get_target_config(target_name)
        fuzzing = cfg.get("fuzzing", {})

        mapping = fuzzing.get("field_whitelist_by_msg", {})
        if msg_type_name and msg_type_name in mapping:
            return mapping[msg_type_name]

        return fuzzing.get("field_whitelist")

    def get_feedback_defs(self, target_name: str):
        cfg = self.get_target_config(target_name)
        return cfg.get("fuzzing", {}).get("feedback", [])

    def ensure_target_dependencies(self, target_name: str) -> None:
        cfg = self.get_target_config(target_name)
        self._ensure_target_installed(cfg)

    def _is_ros_pkg_available(self, ros_pkg: str) -> bool:
        if not ros_pkg:
            return True
        ret = sp.call(
            ["ros2", "pkg", "prefix", ros_pkg],
            stdout=sp.DEVNULL,
            stderr=sp.DEVNULL,
        )
        return ret == 0

    def _run_verify_cmd(self, target_cfg: dict) -> bool:
        verify_cmd = target_cfg.get("lifecycle", {}).get("verify_cmd")
        if not verify_cmd:
            return True
        ret = sp.call(
            verify_cmd,
            shell=True,
            cwd=target_cfg["target_dir"],
            stdout=sp.DEVNULL,
            stderr=sp.DEVNULL,
        )
        return ret == 0

    def _ensure_target_installed(self, target_cfg: dict) -> None:
        lifecycle = target_cfg.get("lifecycle", {})
        ros_pkg = target_cfg.get("basic", {}).get("ros_pkg", "")
        skip_ros_pkg_check = bool(lifecycle.get("skip_ros_pkg_check", False))
        pkg_ready = skip_ros_pkg_check or (not ros_pkg) or self._is_ros_pkg_available(ros_pkg)

        if pkg_ready and self._run_verify_cmd(target_cfg):
            return

        install_rel = lifecycle.get("install_script")
        if not install_rel:
            raise RuntimeError(
                f"target '{target_cfg['name']}' requires ROS package '{ros_pkg}', "
                "but no lifecycle.install_script is configured"
            )

        install_path = self.resolve_target_path(target_cfg, install_rel)
        if not os.path.isfile(install_path):
            raise FileNotFoundError(
                f"install script not found for target '{target_cfg['name']}': {install_path}"
            )

        print(f"[target_manager] package '{ros_pkg}' not found, installing via {install_path}")
        ret = sp.call(
            ["bash", install_path],
            cwd=target_cfg["target_dir"],
        )
        if ret != 0:
            raise RuntimeError(
                f"install script failed for target '{target_cfg['name']}' (exit={ret})"
            )

        if (not skip_ros_pkg_check) and (not self._is_ros_pkg_available(ros_pkg)):
            raise RuntimeError(
                f"target '{target_cfg['name']}' install completed but ROS package '{ros_pkg}' is still missing"
            )
        if not self._run_verify_cmd(target_cfg):
            raise RuntimeError(
                f"target '{target_cfg['name']}' install completed but lifecycle.verify_cmd failed"
            )

    def start_target(self, target_name: str):
        cfg = self.get_target_config(target_name)
        lifecycle = cfg.get("lifecycle", {})
        self._ensure_target_installed(cfg)

        start_cmd = lifecycle.get("start_cmd") or lifecycle.get("exec_cmd")
        if not start_cmd:
            return None

        proc = sp.Popen(
            start_cmd,
            shell=True,
            preexec_fn=os.setpgrp,
            stdout=sp.PIPE,
            stderr=sp.PIPE,
        )
        self.process_map[target_name] = proc

        warmup = float(lifecycle.get("warmup_sec", 0))
        if warmup > 0:
            time.sleep(warmup)

        if proc.poll() is not None:
            stderr = ""
            try:
                stderr = proc.stderr.read().decode("utf-8", errors="ignore")
            except Exception:
                pass
            raise RuntimeError(
                f"target '{target_name}' exited during startup (code={proc.returncode}). "
                f"stderr:\n{stderr[-800:]}"
            )

        return proc

    def stop_target(self, target_name: str) -> None:
        cfg = self.get_target_config(target_name)
        lifecycle = cfg.get("lifecycle", {})

        stop_cmd = lifecycle.get("stop_cmd")
        if stop_cmd:
            sp.call(stop_cmd, shell=True)

        proc = self.process_map.get(target_name)
        if proc is not None:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            except Exception:
                pass
            finally:
                self.process_map.pop(target_name, None)
