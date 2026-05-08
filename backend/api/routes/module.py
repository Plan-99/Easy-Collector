# -*- coding: utf-8 -*-
"""
Module/Extension API routes.

Launcher가 설치 시 `${EASYTRAINER_DATA_DIR}/project/modules/<id>.json` 으로
저장하는 manifest 들을 단일 진실원천으로 사용한다. 프론트엔드는 이 응답을
Pinia 에 캐시해두고 `modules.has('sam3')` 같은 헬퍼로 extension-specific UI
의 노출 여부를 결정한다.
"""
from flask import Blueprint, jsonify

from ...configs.module_loader import load_installed_modules


module_bp = Blueprint('module', __name__)


@module_bp.route('/modules/installed', methods=['GET'])
def installed_modules():
    return jsonify({'modules': load_installed_modules()})
