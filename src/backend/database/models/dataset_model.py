from orator import Model, SoftDeletes
from orator.orm import belongs_to
from .task_model import Task
from orator.orm import accessor
import os
import h5py
from ...configs.global_configs import DATASET_DIR

class Dataset(Model, SoftDeletes):

    __fillable__ = [
        'name',
        'task_id',
    ]
    
    __appends__ = [
        'robot_ids',
        'sensor_ids',
        'episode_len',
        # 'episode_num',
        'episodes',
        'hdf5_metadata',
    ]

    __casts__ = {
        'sensor_ids': 'json',
        'hdf5_metadata': 'json',
    }
    
    __timestamps__ = True


    @belongs_to
    def task(self):
        return Task
    
    @accessor
    def robot_ids(self):
        return self.task.robot_ids if self.task else None
    
    @accessor
    def sensor_ids(self):
        return self.task.sensor_ids if self.task else None
    
    @accessor
    def episode_len(self):
        return self.task.episode_len if self.task else None
    
    # @accessor
    # def episode_num(self):
    #     dataset_dir = DATASET_DIR

    #     dataset_path = os.path.join(dataset_dir, str(self.id))
    #     if not os.path.exists(dataset_path):
    #         return 0
        
    #     # Count the number of episodes in the dataset directory
    #     return len([name for name in os.listdir(dataset_path)])
    
    @accessor
    def episodes(self):
        folder_path = os.path.join(DATASET_DIR, str(self.id))
        if not os.path.exists(folder_path) or not os.path.isdir(folder_path):
            return {'status': 'error', 'message': 'Folder not found'}, 404

        files = [{ 'name': f } for f in os.listdir(folder_path) if os.path.isfile(os.path.join(folder_path, f))]
        files = sorted(
            files, key=lambda x: os.path.getmtime(os.path.join(folder_path, x['name'])), reverse=False
        )
        return files
    
    @accessor
    def hdf5_metadata(self):
        folder_path = os.path.join(DATASET_DIR, str(self.id))
        if not os.path.exists(folder_path) or not os.path.isdir(folder_path):
            return {'status': 'error', 'message': 'Folder not found'}, 404

        metadata = {}
        if os.listdir(folder_path):
            first_file = os.listdir(folder_path)[0]
            hdf5_path = os.path.join(folder_path, first_file)
            with h5py.File(hdf5_path, 'r') as f:
                sensor_names = [name for name in f["observations/images"].keys()]
                robot_names = [name for name in f["observations/qpos"].keys()]

                metadata['sensors'] = sensor_names
                metadata['robots'] = robot_names
        else:
            metadata['sensors'] = []
            metadata['robots'] = []

        return metadata