"""rosbag2(sqlite3) 에서 필요한 토픽만 읽는다. ROS 2 환경(source /opt/ros/humble/setup.bash) 필요.

rosbag2_py 대신 sqlite3 를 직접 연다 — 토픽을 골라 읽을 때 훨씬 빠르고, 녹화 끝 처리(metadata)가
덜 된 bag 도 열린다. 단, **녹화 중인 bag 은 읽지 않는다**(녹화기와 디스크·CPU 를 다툰다).
"""
import glob
import os
import sqlite3

try:
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError as e:  # pragma: no cover
    raise SystemExit('ROS 2 환경이 필요하다:  source /opt/ros/humble/setup.bash') from e


def db_path(bag_dir):
    dbs = sorted(glob.glob(os.path.join(bag_dir, '*.db3')))
    if not dbs:
        raise SystemExit(f'{bag_dir}: .db3 가 없다(sqlite3 bag 만 읽는다)')
    return dbs


def read(bag_dir, topics, t_from=None, t_to=None):
    """(토픽, 녹화 시각 s, 메시지) 를 시간 순으로. t_from/t_to 는 녹화 시각(s) 범위."""
    for db in db_path(bag_dir):
        con = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
        tmap = {tid: (name, get_message(typ)) for tid, name, typ in con.execute('select id, name, type from topics')
                if name in topics}
        if not tmap:
            continue
        q = 'select topic_id, timestamp, data from messages where topic_id in (%s)' % ','.join(map(str, tmap))
        args = []
        if t_from is not None:
            q += ' and timestamp >= ?'
            args.append(int(t_from * 1e9))
        if t_to is not None:
            q += ' and timestamp <= ?'
            args.append(int(t_to * 1e9))
        for tid, ts, data in con.execute(q + ' order by timestamp', args):
            name, cls = tmap[tid]
            yield name, ts * 1e-9, deserialize_message(data, cls)
        con.close()
