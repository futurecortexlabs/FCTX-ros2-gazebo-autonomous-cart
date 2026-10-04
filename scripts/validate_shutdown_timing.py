#!/usr/bin/env python3
"""起動直後・準備直後・切替・画面終了の実プロセス停止を反復検証する。

外部のROS/Gazeboへ接続しないよう、実行時は専用ドメインとpartitionを指定する。
強制終了やGazeboの異常終了が起きたケースを成功として記録しない。
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import time

os.environ['QT_QPA_PLATFORM'] = 'offscreen'
os.environ.setdefault('ROS_DOMAIN_ID', '42')
os.environ.setdefault('GZ_PARTITION', 'ros2_gazebo_loop')

import rclpy
from rclpy.signals import SignalHandlerOptions
from PyQt5 import QtCore, QtWidgets
from course_selector import CoursePanel, PanelNode, COURSES, ROOT, configure_japanese_font


def group_members(pgid):
    """所有launchと同じプロセス群に、実行中の子が残っていないことを調べる。"""
    result = []
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            if os.getpgid(int(proc.name)) != pgid:
                continue
            state = proc.joinpath('stat').read_text().rsplit(')', 1)[1].split()[0]
            if state != 'Z':
                result.append(int(proc.name))
        except (ProcessLookupError, FileNotFoundError, PermissionError):
            pass
    return result


def verify_launch(launch, pgid, require_zero=True):
    """切替前後それぞれのlaunchと、その所有プロセス群を照合する。"""
    code = launch.poll()
    remaining = group_members(pgid)
    assert code is not None, ('Owned launch still running', pgid)
    assert not require_zero or code == 0, ('Owned launch exit', pgid, code)
    assert not remaining, ('Owned process group remains', pgid, remaining)
    return dict(pgid=pgid, exit_code=code, remaining=remaining)


def write_record(path, results, expected, completed=False, failure=None, metadata=None):
    """全条件と最終cleanupを完了するまで、途中の合格を全体合格にしない。"""
    passed = bool(completed and failure is None and len(results) == expected
                  and all(case.get('passed') and case.get('cleanup_completed') for case in results))
    record = dict(metadata or {}, all_passed=passed, completed=completed,
                  expected_cases=expected, cases=results)
    if failure is not None:
        record['failure'] = failure
    path.write_text(json.dumps(record, ensure_ascii=False, indent=2)+'\n')
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--include-gui', action='store_true')
    parser.add_argument('--output', type=Path, default=ROOT/'logs/shutdown_timing_validation.json')
    args = parser.parse_args()
    if args.repeats < 1:
        parser.error('--repeats must be positive')
    ROOT.joinpath('logs').mkdir(exist_ok=True)
    results = []
    warehouse = next(i for i, c in enumerate(COURSES) if c[1] == 'warehouse_course.sdf')
    oval = next(i for i, c in enumerate(COURSES) if c[1] == 'oval_course.sdf')
    logfile = ROOT/'logs/course_selector.log'
    resultfile = args.output.resolve()
    resultfile.parent.mkdir(parents=True, exist_ok=True)
    paths = list((ROOT/'scripts').glob('*.py')) + list((ROOT/'launch').glob('*.py'))
    paths += list((ROOT/'config').glob('*.yaml')) + list(ROOT.glob('*.sh'))
    for stem in ('warehouse_course', 'oval_course'):
        paths += [ROOT/'worlds'/(stem+'.sdf')] + list((ROOT/'maps'/stem).glob('*'))
    metadata = dict(ros_domain_id=os.environ['ROS_DOMAIN_ID'], gz_partition=os.environ['GZ_PARTITION'],
                    source_files_sha256={p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
                                         for p in sorted(set(paths)) if p.is_file()})

    def until(test, panel, seconds=100):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            app.processEvents()
            if test():
                return
            time.sleep(.02)
        raise TimeoutError(panel.course_status.text())

    def wait(panel, seconds):
        began = time.monotonic()
        until(lambda: time.monotonic()-began >= seconds, panel, seconds+2)

    def ready(panel):
        until(lambda: panel.process is not None and not panel.starting
              and panel.stopping_since is None, panel, 90)
        assert panel.node.mode == 'manual' and abs(panel.node.actual_speed) < .02

    def check_native_log(output, events):
        bad = [line for line in output.splitlines()
               if (re.search(r'\[gz-\d+\]', line)
                   and any(s in line for s in ('process has died', 'failed to terminate')))
               or 'Segmentation fault' in line or 'Cannot shutdown' in line]
        assert not bad, '\n'.join(bad)
        assert not any(name in ('SIGTERM', 'SIGKILL') for name, _ in events), events

    cases = [('starting_cancel', delay, False, 'simulation') for delay in (0., .2, 2.)]
    cases += [('ready_immediate', 0., False, 'localization') for _ in range(args.repeats)]
    cases += [('ready_wait_three', 3., False, 'localization'),
              ('pending_switch', 0., False, 'localization')]
    if args.include_gui:
        cases += [('ready_immediate_gui', 0., True, 'localization')]

    write_record(resultfile, results, len(cases), metadata=metadata)
    app = QtWidgets.QApplication([])
    app.setQuitOnLastWindowClosed(False)
    configure_japanese_font(app)
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    complete = False
    failure = None
    try:
        for number, (scenario, delay, gui, source) in enumerate(cases, 1):
            case = dict(number=number, scenario=scenario, delay_seconds=delay,
                        gui=gui, source=source, passed=False, cleanup_completed=False)
            results.append(case)
            write_record(resultfile, results, len(cases), metadata=metadata)
            node, panel = None, None
            offset = logfile.stat().st_size if logfile.exists() else 0
            began = time.monotonic()
            owned = []
            try:
                node = PanelNode()
                panel = CoursePanel(node, simulation_gui=gui)
                panel.show()
                panel.combo.setCurrentIndex(warehouse)
                panel.pose_source.setCurrentIndex(panel.pose_source.findData(source))
                panel.switch_course()
                launch = panel.process
                assert launch is not None
                launch_pgid = launch.pid
                owned.append((launch, launch_pgid))
                if scenario == 'starting_cancel':
                    wait(panel, delay)
                else:
                    ready(panel)
                    wait(panel, delay)
                stop_began = time.monotonic()
                if scenario == 'pending_switch':
                    panel.combo.setCurrentIndex(oval)
                    panel.switch_course()
                    until(lambda: panel.process is not None and panel.process is not launch,
                          panel)
                    events = list(panel.shutdown_events)
                    verify_launch(launch, launch_pgid)
                    assert panel.shutdown_request is None
                    # 次コースも独立した所有launchとして、終了コードとPGIDを確認する。
                    second_launch = panel.process
                    owned.append((second_launch, second_launch.pid))
                    ready(panel)
                    panel.close()
                    until(lambda: panel.process is None and panel.shutdown_request is None, panel)
                    events += list(panel.shutdown_events)
                else:
                    # closeEventは子の終了が確定するまでウィンドウ終了を保留する。
                    panel.close()
                    until(lambda: panel.process is None and panel.shutdown_request is None, panel)
                    events = list(panel.shutdown_events)
                for handle, pgid in owned:
                    until(lambda: not group_members(pgid), panel, 5)
                    verify_launch(handle, pgid, require_zero=scenario != 'starting_cancel')
                assert panel.shutdown_request is None
                output = logfile.read_bytes()[offset:].decode('utf-8', errors='replace')
                check_native_log(output, events)
                if scenario != 'starting_cancel':
                    assert any(name == 'pause_applied' for name, _ in events), output[-4000:]
                    assert any(name == 'SIGINT' for name, _ in events), events
                    assert len(re.findall(r'\[gz-\d+\]: process has finished cleanly', output)) >= len(owned), output[-4000:]
                case.update(
                    elapsed_seconds=time.monotonic()-began,
                    shutdown_seconds=time.monotonic()-stop_began,
                    launch_exit_code=launch.returncode,
                    events=[dict(event=name, seconds=stamp-stop_began) for name, stamp in events],
                    owned_launches=[verify_launch(handle, pgid, require_zero=scenario != 'starting_cancel')
                                    for handle, pgid in owned],
                    native_processes_remaining=sorted({pid for _, pgid in owned for pid in group_members(pgid)}))
            except BaseException as exc:
                case['error'] = f'{type(exc).__name__}: {exc}'
                raise
            finally:
                try:
                    if panel is not None:
                        panel.close()
                        until(lambda: panel.process is None and panel.shutdown_request is None, panel)
                        for handle, pgid in owned:
                            until(lambda: not group_members(pgid), panel, 5)
                            verify_launch(handle, pgid, require_zero=scenario != 'starting_cancel')
                        panel.deleteLater()
                        QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.DeferredDelete)
                    if node is not None:
                        node.destroy_node()
                    case['cleanup_completed'] = True
                except BaseException as exc:
                    case['cleanup_failure'] = f'{type(exc).__name__}: {exc}'
                    raise
                finally:
                    write_record(resultfile, results, len(cases), metadata=metadata)
            case['passed'] = True
            write_record(resultfile, results, len(cases), metadata=metadata)
            print('PASS shutdown timing', case, flush=True)
        complete = True
    except BaseException as exc:
        failure = f'{type(exc).__name__}: {exc}'
        raise
    finally:
        try:
            rclpy.try_shutdown()
        except BaseException as exc:
            complete = False
            failure = f'Final cleanup {type(exc).__name__}: {exc}'
            raise
        finally:
            write_record(resultfile, results, len(cases), completed=complete,
                         failure=failure, metadata=metadata)
    print('ALL SHUTDOWN TIMING CASES PASSED', len(results), flush=True)


if __name__ == '__main__':
    main()
