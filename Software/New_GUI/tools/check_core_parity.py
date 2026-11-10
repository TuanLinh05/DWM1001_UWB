"""Compare full Python/C++ trajectories, all range/attitude flags and counters.

Compile estimator_parity.cpp with g++/Eigen, run it on each exported bundle,
then: python tools/check_core_parity.py --suite BUNDLE_ROOT [--exe EXECUTABLE]
--exe is convenient on Linux; on Windows the same binary can be run separately in WSL.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys

import numpy as np

from flight_log_io import write_json


def _read(path, columns):
    if not path.is_file():
        raise ValueError(f'Missing parity output: {path}')
    return np.loadtxt(path,delimiter=',',ndmin=2) if path.stat().st_size else np.empty((0,columns))


def compare_bundle(path):
    path = Path(path)
    py = _read(path/'python_states.csv',247)
    cpp = _read(path/'cpp/cpp_states.csv',247)
    if py.shape!=cpp.shape or py.shape[1]!=247:
        raise ValueError(f'State shape mismatch in {path}: {py.shape}, {cpp.shape}')
    flags = _read(path/'python_updates.csv',6)
    actual_flags = _read(path/'cpp/cpp_updates.csv',6)
    flags_equal = flags.shape==actual_flags.shape and np.array_equal(flags[:,:4],actual_flags[:,:4])
    expected = json.loads((path/'python_summary.json').read_text())
    actual = json.loads((path/'cpp/cpp_summary.json').read_text())
    counters_equal = expected==actual
    if len(py):
        position = float(np.max(np.abs(py[:,1:4]-cpp[:,1:4])))
        # Frobenius chord -> angle is stable near zero, unlike acos(trace).
        R,Rc = py[:,7:16].reshape(-1,3,3),cpp[:,7:16].reshape(-1,3,3)
        rotation = float(np.max(2*np.arcsin(np.clip(np.linalg.norm(R-Rc,axis=(1,2))/(2*np.sqrt(2)),0,1))))
        covariance = float(np.max(np.abs(py[:,22:]-cpp[:,22:]))/max(float(np.max(np.abs(py[:,22:]))),1e-300))
        times_equal = bool(np.array_equal(py[:,0],cpp[:,0]))
    else:
        position=rotation=covariance=0.;times_equal=True
    nis_delta = float(np.max(np.abs(flags[:,4]-actual_flags[:,4]))) if flags_equal and len(flags) else 0.
    passed = flags_equal and counters_equal and times_equal and position<=1e-6 and rotation<=1e-6 and covariance<=1e-6
    return dict(passed=passed,states=len(py),updates=len(flags),position_max_m=position,rotation_max_rad=rotation,
                covariance_relative_max=covariance,flags_identical=flags_equal,counters_identical=counters_equal,
                output_times_identical=times_equal,nis_max_difference=nis_delta,expected_summary=expected,cpp_summary=actual)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--suite',required=True,type=Path)
    parser.add_argument('--exe',type=Path)
    args = parser.parse_args(argv)
    try:
        groups = json.loads((args.suite/'suite.json').read_text())
        reports = {}
        for group,cases in sorted(groups.items()):
            reports[group] = {}
            for name in cases:
                if name=='frames':
                    folder=args.suite/group
                    if args.exe:subprocess.run([str(args.exe.resolve()),'--frames',str(folder/'frames.csv'),str(folder/'cpp_frames.csv')],check=True)
                    reference=_read(folder/'frames.csv',17)[:,6:]
                    actual=_read(folder/'cpp_frames.csv',11)
                    delta=float(np.max(np.abs(reference-actual))) if reference.shape==actual.shape else float('inf')
                    result=dict(passed=delta<=1e-6,max_difference=delta,cases=len(reference))
                else:
                    folder=args.suite/group/name
                    if args.exe:subprocess.run([str(args.exe.resolve()),str(folder),str(folder/'cpp')],check=True)
                    result=compare_bundle(folder)
                reports[group][name]=result
                print(f'{group}/{name}: {result}',flush=True)
        checks=[case for group in reports.values() for case in group.values()]
        report=dict(groups=reports,all_passed=all(c['passed'] for c in checks),
                    maximums={k:max(c.get(k,0.) for c in checks) for k in ('position_max_m','rotation_max_rad','covariance_relative_max','nis_max_difference')},
                    thresholds=dict(position_m=1e-6,rotation_rad=1e-6,covariance_relative=1e-6))
        write_json(args.suite/'parity_report.json',report)
        print('All passed:',report['all_passed'],'Maximums:',report['maximums'])
        return 0 if report['all_passed'] else 1
    except (OSError,ValueError,RuntimeError,subprocess.CalledProcessError) as exc:
        print(f'check_core_parity: {exc}',file=sys.stderr)
        return 2


if __name__=='__main__':
    sys.exit(main())
