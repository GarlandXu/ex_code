import os
import torch
import gpytorch
import pandas as pd
import glob
import numpy as np
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split

class GPR(gpytorch.models.ExactGP):
    def __init__(self, train_x, train_y, likelihood):
        super(GPR, self).__init__(train_x, train_y, likelihood)
        self.mean_module = gpytorch.means.ConstantMean()
        self.covar_module = gpytorch.kernels.ScaleKernel(gpytorch.kernels.RBFKernel())

    def forward(self, x):
        mean_x = self.mean_module(x)
        covar_x = self.covar_module(x)
        return gpytorch.distributions.MultivariateNormal(mean_x, covar_x)


def load_and_preprocess_data(folder_path):
    """Load and preprocess data by normalizing time for each file and normalizing quaternions."""
    file_pattern = os.path.join(folder_path, "demo_*.txt")
    all_files = glob.glob(file_pattern)
    data_frames = []

    for file in all_files:
        df = pd.read_csv(file, sep=",")
        df['normalized_time'] = (df['time'] - df['time'].min()) / (df['time'].max() - df['time'].min())
        data_frames.append(df)

    combined_data = pd.concat(data_frames, ignore_index=True)
    quaternion_columns = ['psm2_ox', 'psm2_oy', 'psm2_oz', 'psm2_ow']
    combined_data[quaternion_columns] = combined_data[quaternion_columns].apply(
        lambda row: row / np.linalg.norm(row), axis=1
    )
    return combined_data


def train(x, y):
    train_x = torch.tensor(x, dtype=torch.float32)
    train_y = torch.tensor(y, dtype=torch.float32)
    likelihood = gpytorch.likelihoods.GaussianLikelihood()
    model = GPR(train_x, train_y, likelihood)

    model.train()
    likelihood.train()
    optimizer = torch.optim.Adam([
    {'params': model.parameters()}, 
], lr=0.01)
    mll = gpytorch.mlls.ExactMarginalLogLikelihood(likelihood, model)

    training_iter = 50
    for i in range(training_iter):
        optimizer.zero_grad()
        output = model(train_x)
        loss = -mll(output, train_y)
        loss.backward()
        print('Iter %d/%d - Loss: %.3f   lengthscale: %.3f   noise: %.3f' % (
        i + 1, training_iter, loss.item(),
        model.covar_module.base_kernel.lengthscale.item(),
        model.likelihood.noise.item()
    ))
        optimizer.step()

    return model, likelihood


def plot_predictions(time, original, predicted, component):
    plt.figure(figsize=(10, 6))
    plt.plot(time, original, 'k*', label='Original Data')
    plt.plot(time, predicted, 'r', label='Prediction')
    plt.title(f"Gaussian Process Regression for {component}")
    plt.xlabel("Normalized Time")
    plt.ylabel(f"{component} value")
    plt.legend()
    plt.show()


def main():
    folder_path = "/Users/garlandxu/Desktop/lfd/data"  
    data = load_and_preprocess_data(folder_path)

    time = data['time'].values
    quaternions = data[['psm2_ox', 'psm2_oy', 'psm2_oz', 'psm2_ow']].values

    models = {}
    likelihoods = {}
    predictions = {}

    for i, component in enumerate(['x', 'y', 'z', 'w']):
        print(f"Training GPR model for quaternion component: {component}")
        y = quaternions[:, i]
        model, likelihood = train(time, y)
        models[component] = model
        likelihoods[component] = likelihood

       
        model.eval()
        likelihood.eval()
        test_x = torch.tensor(time, dtype=torch.float32)
        with torch.no_grad(), gpytorch.settings.fast_pred_var():
            observed_pred = likelihood(model(test_x))
            predictions[component] = observed_pred.mean.numpy()

        plot_predictions(time, y, predictions[component], f"psm2_o{component}")




if __name__ == "__main__":
    main()