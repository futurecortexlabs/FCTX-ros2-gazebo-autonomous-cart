"""実試験が所有するlaunchの終了を、子プロセスと強制終了も含めて照合する。"""
import os
import psutil


def owned_snapshot(launch):
    """別sessionへ移った子も追えるよう、終了前にPIDと生成時刻を記録する。"""
    try:
        root = psutil.Process(launch.pid)
        processes = [root, *root.children(recursive=True)]
    except psutil.NoSuchProcess:
        return {}
    identities = {}
    for process in processes:
        try:
            identities[process.pid] = process.create_time()
        except psutil.NoSuchProcess:
            pass
    return identities


def remaining_owned(launch_pid, identities):
    """PID再利用を避け、同じ所有groupか捕捉済みの子だけを調べる。信号は送らない。"""
    remaining = []
    for process in psutil.process_iter():
        try:
            same_group = os.getpgid(process.pid) == launch_pid
            known_child = (process.pid in identities
                           and process.create_time() == identities[process.pid])
            if (same_group or known_child) and process.status() != psutil.STATUS_ZOMBIE:
                remaining.append(process.pid)
        except (psutil.NoSuchProcess, psutil.AccessDenied, ProcessLookupError, PermissionError):
            pass
    return sorted(remaining)


def verify_clean_stop(launch, shutdown_events, identities, log_segment):
    """exit0でもSIGTERM/SIGKILLや残存childがあれば不合格にする。"""
    assert launch.poll() == 0, ('Owned launch exit', launch.returncode)
    assert not any(name in ('SIGTERM', 'SIGKILL') for name, _ in shutdown_events), shutdown_events
    assert not any(word in log_segment for word in (
        'process has died', 'failed to terminate', 'Segmentation fault', 'Cannot shutdown',
    )), log_segment[-6000:]
    remaining = remaining_owned(launch.pid, identities)
    assert not remaining, ('Owned processes remain', remaining)
    return remaining
