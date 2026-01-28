import os
import shutil
from ...configs.global_configs import DATASET_DIR
from .record_episode import get_auto_index

def merge_dataset(source_dataset, target_datasets):
    """
    Merges episodes from target datasets into a source dataset.

    :param source_dataset: The Dataset model instance for the source.
    :param target_datasets: A list of Dataset model instances to merge from.
    """
    source_path = os.path.join(DATASET_DIR, str(source_dataset.id))
    if not os.path.isdir(source_path):
        os.makedirs(source_path)

    print(f"Merging datasets into source dataset: {source_dataset.name} (ID: {source_dataset.id})")

    for i, target_dataset in enumerate(target_datasets):
        if target_dataset.id == source_dataset.id:
            continue

        target_path = os.path.join(DATASET_DIR, str(target_dataset.id))
        print(f"Processing target dataset: {target_dataset.name} (ID: {target_dataset.id}) from path: {target_path}")

        if not os.path.isdir(target_path):
            print(f"  - Warning: Target directory {target_path} not found. Skipping.")
            continue

        # Move episode files
        try:
            episode_files = [f for f in os.listdir(target_path) if f.startswith('episode_') and f.endswith('.hdf5')]
            print(f"  - Found {len(episode_files)} episodes to move.")

            for episode_file in episode_files:
                old_path = os.path.join(target_path, episode_file)
                
                # Get new index in the source directory
                new_index = get_auto_index(source_path)
                new_filename = f"episode_{new_index}.hdf5"
                new_path = os.path.join(source_path, new_filename)
                
                print(f"    - Moving {old_path} to {new_path}")

                shutil.move(old_path, new_path)

            # Delete target dataset from DB
            print(f"  - Deleting dataset record for: {target_dataset.name} (ID: {target_dataset.id})")
            target_dataset.delete()

            # Remove empty target directory
            if not os.listdir(target_path):
                print(f"  - Removing empty directory: {target_path}")
                os.rmdir(target_path)
            else:
                print(f"  - Warning: Target directory {target_path} is not empty after moving files. Not removing.")

        except Exception as e:
            print(f"An error occurred while processing target dataset {target_dataset.name}: {e}")
            # For now, we'll just print the error and continue
    
    print("Dataset merge process completed.")