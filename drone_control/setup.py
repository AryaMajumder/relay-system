from glob import glob

from setuptools import setup

package_name = 'drone_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, f'{package_name}.relay_bt', f'{package_name}.config'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/drone_control']),
        ('share/drone_control', ['package.xml']),
        ('share/drone_control/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Drone control package',
    license='MIT-0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ── Data sources ──────────────────────────────────────────────────
            'signal_faker    = drone_control.signal_faker:main',

            # ── Data readers / bridges ────────────────────────────────────────
            'state_bridge    = drone_control.state_bridge:main',

            # ── Radio health layer ────────────────────────────────────────────
            'follower_radio_health_reader = drone_control.follower_radio_health_reader:main',
            'leader_radio_health_reader   = drone_control.leader_radio_health_reader:main',
            'gc_radio_health_reader       = drone_control.gc_radio_health_reader:main',
            'gc_link_observer             = drone_control.gc_link_observer:main',

            # ── Leader-side detection ─────────────────────────────────────────
            'leader_link_detector   = drone_control.leader_link_detector:main',

            # ── Decision layer ────────────────────────────────────────────────
            'capability_assessor       = drone_control.capability_assessor:main',
            'relay_strategy_evaluator  = drone_control.relay_strategy_evaluator:main',
            'relay_decision_authority  = drone_control.relay_decision_authority:main',

            # ── Execution layer ───────────────────────────────────────────────
            'chain_assigner         = drone_control.chain_assigner:main',
            'strategy_executor      = drone_control.strategy_executor:main',
            'relay_position_tracker = drone_control.relay_position_tracker:main',
            'relay_mover            = drone_control.relay_mover:main',
            'px4_agent              = drone_control.px4_agent:main',

            # ── Monitoring ────────────────────────────────────────────────────
            'continuous_monitor     = drone_control.continuous_monitor:main',
        ],
    },
)
