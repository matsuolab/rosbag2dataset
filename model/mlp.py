import torch 
import torch.nn as nn

class MLP(nn.Module):

    def __init__(self, input_dims=64, hidden_dims=[128,256], output_dims=10):
        super().__init__()
        hidden_dims = [input_dims] + hidden_dims
        layers = []
        for idx in range(len(hidden_dims)-1):
            layers += [
                nn.Linear(hidden_dims[i], hidden_dims[i+1]),
                nn.ReLU(inplace=True)
            ]
        self.layers = nn.Sequential(*layers)

    def forward(self, x):
        return self.layers(x)