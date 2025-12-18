"""
Launcher that allows running backend.api.app without ROS 2 (rclpy) installed,
and avoids duplicate route errors in legacy app layout during import.

Use with env: EC_BACKEND_ENTRY=backend.api.noros_launcher
"""
import os
import sys
import types


def _install_rclpy_stub():
    if 'rclpy' in sys.modules:
        return
    rclpy = types.ModuleType('rclpy')
    def init(*args, **kwargs):
        pass
    rclpy.init = init
    node_mod = types.ModuleType('rclpy.node')
    class Node:  # minimal placeholder
        def __init__(self, *args, **kwargs):
            pass
    node_mod.Node = Node
    rclpy.node = node_mod
    sys.modules['rclpy'] = rclpy
    sys.modules['rclpy.node'] = node_mod


def _patch_flask_safe_add_url_rule():
    try:
        import flask
    except Exception:
        return
    _orig = flask.Flask.add_url_rule
    def _safe_add(self, rule, endpoint=None, view_func=None, **options):
        try:
            ep = endpoint or (getattr(view_func, "__name__", None))
            if ep and ep in self.view_functions:
                # Skip duplicate endpoint registration (legacy double definitions)
                return None
        except Exception:
            pass
        return _orig(self, rule, endpoint=endpoint, view_func=view_func, **options)
    flask.Flask.add_url_rule = _safe_add


def main():
    _install_rclpy_stub()
    _patch_flask_safe_add_url_rule()
    # Import and run the real app now that rclpy is stubbed and Flask is patched
    from . import app as real
    from .process_manager import ProcessManager
    debug = os.environ.get('EC_DEBUG', '0') == '1'
    # Minimal init expected by top-level routes in app.py
    try:
        real.app.node = None
        real.app.pm = ProcessManager(real.socketio, debug=debug)
        real.app.agents = {}
    except Exception:
        pass
    # Run with Werkzeug allowed (test/dev only)
    real.socketio.run(
        real.app,
        host='0.0.0.0',
        port=5000,
        debug=debug,
        use_reloader=debug,
        allow_unsafe_werkzeug=True,
    )


if __name__ == '__main__':
    main()
