"""Model definitions for SO-100 imitation policies."""

from __future__ import annotations

import abc
from typing import Literal, TypeAlias

import torch
import torch.nn as nn
import torch.nn.functional as F


class BasePolicy(nn.Module, metaclass=abc.ABCMeta):
    """Base class for action chunking policies."""

    def __init__(self, state_dim: int, action_dim: int, chunk_size: int) -> None:
        super().__init__()
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.chunk_size = chunk_size

    @abc.abstractmethod
    def compute_loss(self, state: torch.Tensor, action_chunk: torch.Tensor) -> torch.Tensor:
        """Compute training loss for a batch."""
        raise NotImplementedError

    @abc.abstractmethod
    def sample_actions(self, state: torch.Tensor) -> torch.Tensor:
        """Generate a chunk of actions with shape (batch, chunk_size, action_dim)."""
        raise NotImplementedError

class MLP(nn.Module):
    def __init__(
            self, 
            d_ff: int, 
            depth: int, 
    ) -> None:
        
        super().__init__()
       
        layers: list[nn.Module] = []
        for _ in range(depth):
            layers.extend([
                nn.Linear(d_ff, d_ff),
                nn.GELU(),
            ])

        self.mlp = nn.Sequential(*layers)
    def forward(self, x:torch.Tensor) -> torch.Tensor:
        return self.mlp(x)

# TODO: Students implement ObstaclePolicy here.
class ObstaclePolicy(BasePolicy):
    """Predicts action chunks with an MSE loss.

    A simple MLP that maps a state vector to a flat action chunk
    (chunk_size * action_dim) and reshapes to (B, chunk_size, action_dim).
    """
    def __init__(
        self,
        state_dim: int,
        action_dim: int,
        chunk_size: int,
        d_model: int = 128,
        depth: int = 2,
    ) -> None:
        super().__init__(state_dim, action_dim, chunk_size)
        
        self.net = nn.Sequential(
            nn.Linear(state_dim, d_model),
            MLP(d_model, depth),
            nn.Linear(d_model, chunk_size * action_dim),
            )       
    def forward(self, state: torch.Tensor) -> torch.Tensor:
        """Return predicted action chunk of shape (B, chunk_size, action_dim)."""
        x = self.net(state)
        return x.reshape(state.shape[0], self.chunk_size, self.action_dim)
        

    def compute_loss(self, state: torch.Tensor, action_chunk: torch.Tensor) -> torch.Tensor:
        pred_action_chunk = self(state)
        return nn.MSELoss()(pred_action_chunk, action_chunk)

    def sample_actions(self, state: torch.Tensor) -> torch.Tensor:
        return self(state)


# TODO: Students implement MultiTaskPolicy here.
class MultiTaskPolicy(BasePolicy):
    """Goal-conditioned policy for the multicube scene."""

    def compute_loss(self, state: torch.Tensor, action_chunk: torch.Tensor) -> torch.Tensor:
        raise NotImplementedError

    def sample_actions(self, state: torch.Tensor) -> torch.Tensor:
        raise NotImplementedError

    def forward(self) -> torch.Tensor:
        """Return predicted action chunk of shape (B, chunk_size, action_dim)."""
        raise NotImplementedError


PolicyType: TypeAlias = Literal["obstacle", "multitask"]


def build_policy(
    policy_type: PolicyType,
    *,
    state_dim: int,
    action_dim: int,
    # TODO,
) -> BasePolicy:
    if policy_type == "obstacle":
        return ObstaclePolicy(
            action_dim=action_dim,
            state_dim=state_dim,
            # TODO: Build with your chosen specifications
        )
    if policy_type == "multitask":
        return MultiTaskPolicy(
            action_dim=action_dim,
            state_dim=state_dim,
            # TODO: Build with your chosen specifications
        )
    raise ValueError(f"Unknown policy type: {policy_type}")
