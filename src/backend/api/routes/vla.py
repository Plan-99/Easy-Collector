from flask import Blueprint, request, current_app, Response
from ..process.test_vla import test_vla

vla_bp = Blueprint('vla', __name__)

@vla_bp.route('/vla:test_vla', methods=['Post'])
def test_vla_():
    data = request.json
    try:
        current_app.pm.start_function(
            func=test_vla,
            node=current_app.node,
            model=data.get('model'),
            policy_obj=data.get('policy'),
            robots=data.get('robots', []),
            sensors=data.get('sensors', []),
            image_size=data.get('image_size'),
            prompt=data.get('prompt', ''),
            socketio_instance=current_app.pm.socketio,
            name=f"test_vla",
        )
        return {'status': 'success', 'message': 'VLA test started'}, 200
    except Exception as e:
        current_app.logger.error(f"Error starting VLA: {e}")
        return Response(status=500)