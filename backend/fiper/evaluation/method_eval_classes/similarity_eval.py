from .base_eval_class import BaseEvalClass
import numpy as np
from typing import Optional, Union
from sklearn import metrics
from sklearn.decomposition import PCA
from sklearn.cluster import KMeans

class SIMILARITYEval(BaseEvalClass):
    def __init__(self, cfg, method_name, device, task_data_path, dataset, **kwargs):
        super().__init__(cfg, method_name, device, task_data_path, dataset, **kwargs)

    def _execute_preprocessing(self):
        self.embedding_dim = self.dataset.get_tensor_shape("obs_embeddings")[-1]

        obs_embeddings = self.dataset.get_subset(subset="calibration", required_tensors="obs_embeddings")[
            "obs_embeddings"
        ]
        self.embeddings = np.array(obs_embeddings)

        self.mean_embeddings = np.mean(self.embeddings, axis=0)

        cov = np.cov(self.embeddings.T)
        cov += np.eye(cov.shape[0]) * 1e-12
        self.invcov_embeddings = np.linalg.inv(cov)

        # Compute the PCA of the embeddings
        self.n_components = self.cfg.principal_components if hasattr(self.cfg, "principal_components") else 10
        self.embedding_pca = PCA(n_components=self.n_components)
        self.compressed_embeddings = self.embedding_pca.fit_transform(self.embeddings)

        # Cluster the compressed embeddings into 64 clusters
        n_clusters = self.cfg.n_clusters if hasattr(self.cfg, "n_clusters") else 64
        n_clusters = min(n_clusters, len(self.compressed_embeddings))
        self.kmeans = KMeans(n_clusters=n_clusters, random_state=42)
        self.kmeans.fit(self.compressed_embeddings)


    def calculate_uncertainty_score(self, rollout_tensor_dict, **kwargs):
        obs_embeddings = rollout_tensor_dict["obs_embeddings"]
        if len(obs_embeddings.shape) > 1:
            obs_embeddings = obs_embeddings[0]
        obs_embedding = np.array(obs_embeddings[-self.embedding_dim :])

        uncertainty_score = self.calculate_distance(obs_embedding)

        return uncertainty_score

    def calculate_distance(self, obs_embedding: np.ndarray) -> float:
        # Compute the Mahalanobis distance to the mean embedding
        if self.cfg.type == "mean":
            uncertainty_score = self.mahal_embedding_mean(obs_embedding)
        elif self.cfg.type == 'closest':
            # Compute the Mahalanobis distance to the closest embedding in the database
            uncertainty_score = self.mahal_embedding_closest(obs_embedding)
        elif self.cfg.type == 'pca_kmeans':
            # Compute the distance to the closest cluster center
            uncertainty_score = self.calculate_cluster_distance(obs_embedding)
        else:
            raise ValueError(f"Unknown uncertainty score type: {self.cfg.type}")

        return uncertainty_score

    def mahal_embedding_mean(self, test_embedding: np.ndarray) -> np.ndarray:
        """Compute the Mahalanobis embedding similarity score to the mean embedding."""

        # Compute the Mahalanobis distance
        embedding_diff = test_embedding - self.mean_embeddings

        mahal_dist = embedding_diff @ self.invcov_embeddings @ embedding_diff.T
        mahal_dist = np.sqrt(mahal_dist)

        return mahal_dist

    def mahal_embedding_closest(self, test_embedding: np.ndarray) -> np.ndarray:
        """Compute the Mahalanobis embedding similarity score to the closest embedding in the database."""

        # Compute the Mahalanobis distance
        embedding_diffs = test_embedding - self.embeddings

        # Compute the Mahalanobis distance to each embedding using self.invcov_embeddings
        mahal_distances = np.sqrt(np.sum(embedding_diffs @ self.invcov_embeddings * embedding_diffs, axis=1))

        # Remove values smaller than 1e-8 (distance to itself during calibration)
        mahal_distances = mahal_distances[mahal_distances > 1e-8]

        min_mahal_dist = np.min(mahal_distances)

        return min_mahal_dist
    
    def calculate_cluster_distance(self, test_embedding: np.ndarray) -> float:
        """Reduce the dimension of the test embedding and compute the distance to the closest cluster."""
        # Reduce the dimension of the test embedding using PCA
        compressed_embedding = self.embedding_pca.transform(test_embedding.reshape(1, -1))

        # Compute the distance to the closest cluster center
        distances = np.linalg.norm(self.kmeans.cluster_centers_ - compressed_embedding, axis=1)
        closest_distance = np.min(distances)

        return closest_distance
