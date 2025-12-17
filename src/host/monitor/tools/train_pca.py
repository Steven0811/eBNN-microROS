import os
import numpy as np
from sklearn.decomposition import PCA
from torchvision import datasets, transforms
import joblib

MODEL_DIR = "/app/models"
MODEL_PATH = os.path.join(MODEL_DIR, "pca_mnist.joblib")

os.makedirs(MODEL_DIR, exist_ok=True)

if os.path.exists(MODEL_PATH):
    print(f"PCA model already exists at {MODEL_PATH}, skip training.")
    exit(0)

transform = transforms.Compose([
    transforms.ToTensor()
])

mnist = datasets.MNIST(
    root="/tmp/data",
    train=True,
    download=True,
    transform=transform
)

X = []
for i in range(10000):
    img, _ = mnist[i]
    X.append(img.view(-1).numpy())

X = np.array(X)

pca = PCA(n_components=50)
pca.fit(X)

joblib.dump(pca, MODEL_PATH)

print(f"PCA trained and saved to {MODEL_PATH}")
