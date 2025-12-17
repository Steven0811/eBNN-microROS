import numpy as np
import joblib

class PCAAnomalyDetector:
    def __init__(self, pca_model_path, threshold):
        self.pca = joblib.load(pca_model_path)
        self.threshold = threshold

    def score(self, image):
        """
        image: numpy array shape (28, 28) or (784,)
        """
        if image.ndim == 2:
            x = image.flatten()
        else:
            x = image

        x = x.reshape(1, -1)

        x_proj = self.pca.transform(x)
        x_recon = self.pca.inverse_transform(x_proj)

        error = np.mean((x - x_recon) ** 2)
        return error

    def is_anomaly(self, image):
        error = self.score(image)
        return error > self.threshold, error
