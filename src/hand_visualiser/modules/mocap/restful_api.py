from flask import Flask, jsonify
import numpy as np
import threading
import time

from manotorch.manolayer import MANOOutput

from modules.enumerates import MoCapFingerType
from .mocap import MoCap

app = Flask(__name__)

# Sample data to be returned
def generate_shape_params():
    mocap = MoCap()
    res: dict[str, list[float]] = dict()
    for hand in [mocap.left_hand, mocap.right_hand]:
        if hand:
            res[hand.hand_id.value] = hand.mano_state.shape_params_cpt.detach().cpu().numpy().tolist()
    return res

def generate_pose_params():
    mocap = MoCap()
    res: dict[str, list[float]]  = dict()
    for hand in [mocap.left_hand, mocap.right_hand]:
        if hand:
            res[hand.hand_id.value] = hand.mano_state.pose_params_cpt.detach().cpu().numpy().tolist()
    return res

def generate_vertices():
    mocap = MoCap()
    res: dict[str, list[list[float]]]  = dict()
    for hand in [mocap.left_hand, mocap.right_hand]:
        if hand:
            res[hand.hand_id.value] = hand.mano_state.cache_mano_output.verts.squeeze(0).T.detach().cpu().numpy()
    return res

def generate_mesh():
    mocap = MoCap()
    res: dict[str, dict[str, list[list[float]] ]]= dict()
    for hand in [mocap.left_hand, mocap.right_hand]:
        if hand:
            res[hand.hand_id.value] = {
                'vertices': hand.mano_state.cache_mano_output.verts.squeeze(0).T.detach().cpu().numpy().tolist(),
                'faces': hand.mano_state.mano_layer.th_faces.cpu().numpy().tolist(),
            }
    return res

def generate_finger_tips():
    mocap = MoCap()
    res: dict[str, dict[str, list[float]]] = dict()
    for hand in [mocap.left_hand, mocap.right_hand]:
        if hand:
            res[hand.hand_id.value] = {
                MoCapFingerType.THUMB: hand.mano_state.cache_mano_output.joints.squeeze()[4].cpu().numpy().tolist(),
                MoCapFingerType.INDEX: hand.mano_state.cache_mano_output.joints.squeeze()[8].cpu().numpy().tolist(),
                MoCapFingerType.MIDDLE: hand.mano_state.cache_mano_output.joints.squeeze()[12].cpu().numpy().tolist(),
                MoCapFingerType.RING: hand.mano_state.cache_mano_output.joints.squeeze()[16].cpu().numpy().tolist(),
                MoCapFingerType.LITTLE: hand.mano_state.cache_mano_output.joints.squeeze()[20].cpu().numpy().tolist(),
            }
    return res

# Define the Flask routes
@app.route('/mano/shape_params', methods=['GET'])
def get_shape_params():
    return jsonify(generate_shape_params())

@app.route('/mano/pose_params', methods=['GET'])
def get_pose_params():
    return jsonify(generate_pose_params())

@app.route('/mano/vertices', methods=['GET'])
def get_vertices():
    return jsonify(generate_vertices())

@app.route('/mano/mesh', methods=['GET'])
def get_mesh():
    return jsonify(generate_mesh())

@app.route('/mano/finger_tips', methods=['GET'])
def get_finger_tips():
    return jsonify(generate_finger_tips())

# Function to run Flask app in a thread
def run_flask_app(port:int=8000):
    app.run(host='0.0.0.0', port=port, threaded=True)

# Run Flask server in a separate thread
if __name__ == '__main__':
    server_thread = threading.Thread(target=run_flask_app)
    server_thread.daemon = True
    server_thread.start()

    # Main thread just sleeps (simulating other tasks)
    try:
        while True:
            time.sleep(10)  # Adjust sleep time if needed
    except KeyboardInterrupt:
        print("Server is shutting down.")