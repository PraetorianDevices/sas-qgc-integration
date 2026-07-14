"""Root-level pytest config.

test_gps_spoof_alert_generator.py matches pytest's default test-discovery
pattern (test_*.py) despite being a `ros2 run` CLI tool, not a test module --
its name is fixed by setup.py's console_scripts entry point
(test_gps_spoof_alert_generator:main), so it can't simply be renamed.
Explicitly excluding it here means pytest never attempts to import it as a
test module, regardless of invocation form or collection order -- it should
only ever be run via `ros2 run mavlink-bridge test_gps_spoof_alert_generator`.
"""

collect_ignore = ['test_gps_spoof_alert_generator.py']
