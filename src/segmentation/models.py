"""Segmentation model definitions and model factory."""

from __future__ import annotations

from typing import Any, Callable, Dict

import torch
from torch import nn
import torch.nn.functional as F


class ConvBlock(nn.Module):
    def __init__(self, in_channels: int, out_channels: int) -> None:
        super().__init__()
        self.block = nn.Sequential(
            nn.Conv2d(in_channels, out_channels, kernel_size=3, padding=1, bias=False),
            nn.BatchNorm2d(out_channels),
            nn.ReLU(inplace=True),
            nn.Conv2d(out_channels, out_channels, kernel_size=3, padding=1, bias=False),
            nn.BatchNorm2d(out_channels),
            nn.ReLU(inplace=True),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.block(x)


class UNet(nn.Module):
    def __init__(self, in_channels: int = 3, num_classes: int = 3, base_channels: int = 32) -> None:
        super().__init__()
        self.enc1 = ConvBlock(in_channels, base_channels)
        self.enc2 = ConvBlock(base_channels, base_channels * 2)
        self.enc3 = ConvBlock(base_channels * 2, base_channels * 4)
        self.enc4 = ConvBlock(base_channels * 4, base_channels * 8)
        self.bottleneck = ConvBlock(base_channels * 8, base_channels * 16)

        self.up4 = nn.ConvTranspose2d(base_channels * 16, base_channels * 8, kernel_size=2, stride=2)
        self.dec4 = ConvBlock(base_channels * 16, base_channels * 8)
        self.up3 = nn.ConvTranspose2d(base_channels * 8, base_channels * 4, kernel_size=2, stride=2)
        self.dec3 = ConvBlock(base_channels * 8, base_channels * 4)
        self.up2 = nn.ConvTranspose2d(base_channels * 4, base_channels * 2, kernel_size=2, stride=2)
        self.dec2 = ConvBlock(base_channels * 4, base_channels * 2)
        self.up1 = nn.ConvTranspose2d(base_channels * 2, base_channels, kernel_size=2, stride=2)
        self.dec1 = ConvBlock(base_channels * 2, base_channels)

        self.head = nn.Conv2d(base_channels, num_classes, kernel_size=1)
        self.pool = nn.MaxPool2d(kernel_size=2, stride=2)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        e1 = self.enc1(x)
        e2 = self.enc2(self.pool(e1))
        e3 = self.enc3(self.pool(e2))
        e4 = self.enc4(self.pool(e3))
        b = self.bottleneck(self.pool(e4))

        d4 = self.dec4(torch.cat([self._match(self.up4(b), e4), e4], dim=1))
        d3 = self.dec3(torch.cat([self._match(self.up3(d4), e3), e3], dim=1))
        d2 = self.dec2(torch.cat([self._match(self.up2(d3), e2), e2], dim=1))
        d1 = self.dec1(torch.cat([self._match(self.up1(d2), e1), e1], dim=1))
        return self.head(d1)

    @staticmethod
    def _match(x: torch.Tensor, reference: torch.Tensor) -> torch.Tensor:
        if x.shape[-2:] == reference.shape[-2:]:
            return x
        return F.interpolate(x, size=reference.shape[-2:], mode="bilinear", align_corners=False)


class TinyFCN(nn.Module):
    """Small encoder-decoder baseline for quick model-swap experiments."""

    def __init__(self, in_channels: int = 3, num_classes: int = 3, base_channels: int = 32) -> None:
        super().__init__()
        self.encoder = nn.Sequential(
            ConvBlock(in_channels, base_channels),
            nn.MaxPool2d(2),
            ConvBlock(base_channels, base_channels * 2),
            nn.MaxPool2d(2),
            ConvBlock(base_channels * 2, base_channels * 4),
        )
        self.decoder = nn.Sequential(
            nn.ConvTranspose2d(base_channels * 4, base_channels * 2, kernel_size=2, stride=2),
            ConvBlock(base_channels * 2, base_channels * 2),
            nn.ConvTranspose2d(base_channels * 2, base_channels, kernel_size=2, stride=2),
            ConvBlock(base_channels, base_channels),
            nn.Conv2d(base_channels, num_classes, kernel_size=1),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        logits = self.decoder(self.encoder(x))
        if logits.shape[-2:] != x.shape[-2:]:
            logits = F.interpolate(logits, size=x.shape[-2:], mode="bilinear", align_corners=False)
        return logits


MODEL_REGISTRY: Dict[str, Callable[..., nn.Module]] = {
    "unet": UNet,
    "tiny_fcn": TinyFCN,
}


def build_model(config: Dict[str, Any], num_classes: int) -> nn.Module:
    """Build a segmentation model from config.

    Supported config shapes:

    model:
      name: unet
      base_channels: 32

    or the legacy top-level:

    base_channels: 32
    """
    model_cfg = config.get("model", {})
    if isinstance(model_cfg, str):
        model_cfg = {"name": model_cfg}
    if not isinstance(model_cfg, dict):
        raise TypeError("model config must be a string or mapping")

    name = str(model_cfg.get("name", "unet")).lower()
    if name not in MODEL_REGISTRY:
        options = ", ".join(sorted(MODEL_REGISTRY))
        raise ValueError(f"Unknown model '{name}'. Available models: {options}")

    params = {
        "in_channels": int(model_cfg.get("in_channels", config.get("in_channels", 3))),
        "num_classes": num_classes,
        "base_channels": int(model_cfg.get("base_channels", config.get("base_channels", 32))),
    }
    return MODEL_REGISTRY[name](**params)
