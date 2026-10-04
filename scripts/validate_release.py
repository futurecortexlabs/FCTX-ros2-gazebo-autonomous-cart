"""公開前のロジック・実行器・新規コピー診断を検証する（Gazeboは起動しない）。

ROS 2 の setup.bash を読み込んだ Python で実行する。
連続運転や地図拡充の実シミュレーションは別の検証記録で確認する。
"""
from __future__ import annotations

import argparse
from contextlib import redirect_stdout
from datetime import datetime, timedelta, timezone
import hashlib
import io
import json
import os
from pathlib import Path
import runpy
import shutil
import subprocess
import sys
import tempfile
from types import SimpleNamespace
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
UNIT_SCRIPTS = (
    "test_optimization.py", "test_loop_core.py", "test_obstacle_avoidance.py",
    "test_goal_navigation.py", "test_dynamic_obstacles.py", "test_occupancy_navigation.py",
    "test_localization_bridge.py", "test_mission_control.py", "test_goal_controller.py",
    "test_manual_panel.py", "test_goal_panel.py", "test_safety_gate.py", "test_manual_mode.py",
    "test_startup_supervision.py", "test_validation_processes.py",
    "tests/test_shutdown_timing.py",
    "tests/test_map_coverage.py",
)
RUNNERS = ("run_acceptance.py", "validate_optimization.py")
JST = timezone(timedelta(hours=9))


def release_sources(root: Path) -> list[Path]:
    """配布するフォルダを列挙し、Gitのないソース配布でも同じ検証を行う。"""
    folders = ("scripts", "launch", "config", "worlds", "maps", "routes", "fonts", "tests", "docs", "validation")
    paths = set()
    for folder in folders:
        directory = root / folder
        if not directory.is_dir():
            continue
        for path in directory.rglob("*"):
            relative = path.relative_to(root)
            if not path.is_file() or any(part in {"__pycache__", "backups", "logs", ".git", "build"} for part in relative.parts):
                continue
            if path.suffix.lower() in {".pyc", ".pyo", ".ttc"}:
                continue
            paths.add(relative)
    for path in root.iterdir():
        if path.is_file() and (path.suffix in {".md", ".sh"} or path.name in {".gitignore", ".gitattributes"} or path.name.startswith("LICENSE")):
            paths.add(path.relative_to(root))
    assert Path("scripts/check_environment.py") in paths, "診断スクリプトがありません"
    return sorted(paths)


def source_manifest(root: Path, paths: list[Path]) -> dict[str, str]:
    """実行時ソースのSHA-256を記録し、過去の結果との取り違えを避ける。"""
    runtime_prefixes = {"scripts", "launch", "config", "worlds", "maps", "routes", "tests", "fonts"}
    return {
        relative.as_posix(): hashlib.sha256((root / relative).read_bytes()).hexdigest()
        for relative in paths
        if relative.parts[0] in runtime_prefixes or relative.suffix == ".sh"
    }


def fresh_copy(root: Path, destination: Path, paths: list[Path]) -> None:
    for relative in paths:
        target = destination / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(root / relative, target)
    assert not (destination / "logs").exists()
    assert not (destination / ".git").exists()
    assert not list((destination / "fonts").glob("*.ttc"))
    references = [p for p in paths if p.parts[:2] == ("tests", "reference")]
    assert references, "最適化の比較用参照ソースがありません"
    assert all((destination / p).is_file() for p in references)


def runner_case(root: Path, runner: str, scenario: str) -> dict:
    """ROS/Gazeboの子を作らず、順次実行器の失敗とタイムアウト伝播を確認。"""
    code = {"success": 0, "failure": 7, "timeout": 124}[scenario]
    calls = []

    def simulate(command, **kwargs):
        calls.append(command)
        if scenario == "timeout":
            raise subprocess.TimeoutExpired(command, kwargs.get("timeout", 1))
        return SimpleNamespace(returncode=code)

    captured = io.StringIO()
    exit_code = 0
    # ROOTが実行データのないコピーを指すため、通常の検証記録を上書きしない。
    with patch.object(subprocess, "run", simulate), patch.object(sys, "argv", [str(root / "scripts" / runner)]), redirect_stdout(captured):
        try:
            runpy.run_path(str(root / "scripts" / runner), run_name="__main__")
        except SystemExit as exc:
            exit_code = 0 if exc.code is None else exc.code
    filename = "acceptance_results.json" if runner == "run_acceptance.py" else "optimization_validation.json"
    results = json.loads((root / "logs" / filename).read_text())
    assert calls and results, captured.getvalue()
    assert exit_code == code, (runner, scenario, exit_code)
    assert all(item["exit_code"] == code for item in results), results
    if scenario != "success":
        assert len(calls) == len(results) == 1, "失敗後の段階が実行されました"
    return {"runner": runner, "scenario": scenario, "passed": True, "exit_code": exit_code, "stages_run": len(calls)}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checks", nargs="+", choices=("units", "runners", "fresh-copy"), default=["units", "runners", "fresh-copy"])
    parser.add_argument("--output", type=Path, default=ROOT / "logs/release_static_validation.json")
    parser.add_argument("--unit-timeout", type=float, default=180)
    args = parser.parse_args()
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    paths = release_sources(ROOT)
    record = {
        "started_at_jst": datetime.now(JST).isoformat(timespec="seconds"),
        "scope": "Existing logic/UI scripts, runner failure propagation, fresh-source diagnostics; no Gazebo startup or endurance claim",
        "requested_checks": args.checks,
        "source_files_sha256": {},
        "tests": [], "runner_cases": [], "fresh_copy": None, "passed": False,
    }

    def save():
        output.write_text(json.dumps(record, ensure_ascii=False, indent=2) + "\n")

    save()
    env = os.environ.copy()
    env["QT_QPA_PLATFORM"] = "offscreen"
    try:
        with tempfile.TemporaryDirectory(prefix="fctx-release-") as temp:
            clone = Path(temp) / "source"
            fresh_copy(ROOT, clone, paths)
            record["source_files_sha256"] = source_manifest(clone, paths)
            save()
            if "units" in args.checks:
                for name in UNIT_SCRIPTS:
                    print("RUN", name, flush=True)
                    relative = Path(name)
                    script = clone / relative if len(relative.parts) > 1 else clone / "scripts" / relative
                    logfile = output.parent / ("release_" + relative.stem + ".log")
                    with logfile.open("w") as log:
                        try:
                            code = subprocess.run([sys.executable, str(script)], cwd=clone, env=env, stdout=log, stderr=subprocess.STDOUT, timeout=args.unit_timeout).returncode
                        except subprocess.TimeoutExpired:
                            code = 124
                    record["tests"].append({"script": name, "exit_code": code})
                    save()
                    if code:
                        raise RuntimeError(f"{name} failed with exit {code}; see {logfile}")
            if "runners" in args.checks:
                for runner in RUNNERS:
                    for scenario in ("success", "failure", "timeout"):
                        record["runner_cases"].append(runner_case(clone, runner, scenario))
                        save()
            if "fresh-copy" in args.checks:
                # 単体試験の生成物を取り除いた別のコピーで「初回診断」を確認する。
                pristine = Path(temp) / "pristine"
                fresh_copy(ROOT, pristine, paths)
                diagnostic = subprocess.run([sys.executable, str(pristine / "scripts/check_environment.py")], cwd=pristine, env=env, capture_output=True, text=True, timeout=45)
                diagnostic_file = pristine / "logs/environment_check.json"
                assert diagnostic_file.is_file(), diagnostic.stderr
                data = json.loads(diagnostic_file.read_text())
                expected_maps = {p.name for p in (pristine / "worlds").glob("*course.sdf")}
                assert len(expected_maps) == 7
                assert diagnostic.returncode == 0 and data["dependencies_ready"], data
                assert set(data["maps"]) == expected_maps and all(x == "ready" for x in data["maps"].values()), data["maps"]
                record["fresh_copy"] = {
                    "passed": True, "diagnostic_exit_code": diagnostic.returncode,
                    "excluded": ["logs", ".git", "backups", "local .ttc fonts"],
                    "logs_created_on_first_run": True,
                    "dependencies_ready": data["dependencies_ready"],
                    "packages": data["packages"], "python": data["python"], "maps": data["maps"],
                    "display_configured": data["display_configured"],
                    "scope": "Clean source copy in the same installed WSL environment; not an independent machine installation",
                    "source_files_sha256": source_manifest(pristine, paths),
                }
                save()
        record["passed"] = True
        record["finished_at_jst"] = datetime.now(JST).isoformat(timespec="seconds")
        save()
        print("ALL REQUESTED RELEASE CHECKS PASSED", output, flush=True)
        return 0
    except Exception as exc:
        record["failure"] = f"{type(exc).__name__}: {exc}"
        record["finished_at_jst"] = datetime.now(JST).isoformat(timespec="seconds")
        save()
        print(record["failure"], file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
