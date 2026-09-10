"""
3B nokta bulutu: cikarma, downsample, PLY export, Plotly gorsellestirme.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Optional

import numpy as np

from . import config

logger = logging.getLogger(__name__)


@dataclass
class PointCloud:
    xyz: np.ndarray            # (N, 3) - cm (olceklenmis)
    rgb: Optional[np.ndarray]  # (N, 3) uint8 veya None

    def __len__(self) -> int:
        return int(self.xyz.shape[0])

    # ------------------------------------------------------------------
    def transform(self, T4x4: np.ndarray) -> "PointCloud":
        """Rigid 4x4 donusum (orn. kamera cercevesi -> robot taban cercevesi)."""
        if len(self) == 0:
            return PointCloud(xyz=self.xyz.copy(),
                              rgb=None if self.rgb is None else self.rgb.copy())
        h = np.c_[self.xyz, np.ones(len(self.xyz))]
        xyz = (np.asarray(T4x4, float) @ h.T).T[:, :3]
        return PointCloud(xyz=xyz, rgb=None if self.rgb is None else self.rgb.copy())

    # ------------------------------------------------------------------
    def voxel_downsample(self, voxel_cm: float) -> "PointCloud":
        if voxel_cm <= 0 or len(self) == 0:
            return self
        keys = np.floor(self.xyz / voxel_cm).astype(np.int64)
        _, idx = np.unique(keys, axis=0, return_index=True)
        idx.sort()
        return PointCloud(
            xyz=self.xyz[idx],
            rgb=None if self.rgb is None else self.rgb[idx],
        )

    def subsample(self, max_points: int) -> "PointCloud":
        n = len(self)
        if n <= max_points or max_points <= 0:
            return self
        sel = np.random.default_rng(0).choice(n, size=max_points, replace=False)
        return PointCloud(
            xyz=self.xyz[sel],
            rgb=None if self.rgb is None else self.rgb[sel],
        )

    # ------------------------------------------------------------------
    def to_ply_bytes(self) -> bytes:
        n = len(self)
        has_rgb = self.rgb is not None
        header = [
            "ply",
            "format ascii 1.0",
            f"element vertex {n}",
            "property float x", "property float y", "property float z",
        ]
        if has_rgb:
            header += ["property uchar red", "property uchar green", "property uchar blue"]
        header.append("end_header")
        lines = ["\n".join(header)]
        for i in range(n):
            x, y, z = self.xyz[i]
            if has_rgb:
                r, g, b = self.rgb[i]
                lines.append(f"{x:.5f} {y:.5f} {z:.5f} {int(r)} {int(g)} {int(b)}")
            else:
                lines.append(f"{x:.5f} {y:.5f} {z:.5f}")
        return ("\n".join(lines) + "\n").encode("ascii")


# ----------------------------------------------------------------------
def extract(
    points_xyz_cm: np.ndarray,
    mask: np.ndarray,
    color_bgr: Optional[np.ndarray] = None,
    only_suitcase: bool = True,
) -> PointCloud:
    """
    points_xyz_cm : (H, W, 3) metrik (cm) 3B harita
    mask          : (H, W) bool valiz maskesi
    color_bgr     : (H, W, 3) rektifiye sol goruntu (opsiyonel renk)
    """
    finite = np.isfinite(points_xyz_cm).all(axis=2)
    sel = finite & (mask if only_suitcase else np.ones_like(mask, bool))
    xyz = points_xyz_cm[sel]

    rgb = None
    if color_bgr is not None and color_bgr.ndim == 3:
        bgr = color_bgr[sel]
        rgb = bgr[:, ::-1].astype(np.uint8)

    pc = PointCloud(xyz=xyz, rgb=rgb)
    if config.POINTCLOUD_VOXEL_CM > 0:
        pc = pc.voxel_downsample(config.POINTCLOUD_VOXEL_CM)
    logger.info("Nokta bulutu: %d nokta", len(pc))
    return pc


# ----------------------------------------------------------------------
def plotly_figure(pc: PointCloud, bbox_cm: Optional[dict] = None,
                  title: str = "3B Nokta Bulutu", unit: str = "cm", scale: float = 1.0):
    """Streamlit'te goster: import plotly. scale: koordinatlari carpar
    (mm bulutu -> cm goruntu icin 0.1)."""
    import plotly.graph_objects as go

    view = pc.subsample(config.POINTCLOUD_MAX_POINTS)
    xyz = view.xyz * float(scale)
    if view.rgb is not None:
        colors = ["rgb(%d,%d,%d)" % tuple(int(c) for c in row) for row in view.rgb]
        marker = dict(size=1.6, color=colors)
    else:
        marker = dict(size=1.6, color=xyz[:, 2], colorscale="Turbo", showscale=True,
                      colorbar=dict(title=f"derinlik ({unit})"))

    data = [go.Scatter3d(
        x=xyz[:, 0], y=xyz[:, 1], z=xyz[:, 2],
        mode="markers", marker=marker, name="nokta bulutu",
    )]

    if bbox_cm:
        data.append(_bbox_trace(bbox_cm))

    fig = go.Figure(data=data)
    fig.update_layout(
        title=title,
        scene=dict(
            xaxis_title=f"X - en ({unit})",
            yaxis_title=f"Y - boy ({unit})",
            zaxis_title=f"Z - derinlik ({unit})",
            aspectmode="data",
        ),
        margin=dict(l=0, r=0, t=30, b=0),
        height=600,
    )
    return fig


def _bbox_trace(bbox_cm: dict):
    import plotly.graph_objects as go

    cx, cy, cz = bbox_cm.get("center", (0, 0, 0))
    w = bbox_cm.get("width", 0) / 2.0
    h = bbox_cm.get("height", 0) / 2.0
    d = bbox_cm.get("depth", 0) / 2.0
    corners = np.array([
        [cx - w, cy - h, cz - d], [cx + w, cy - h, cz - d],
        [cx + w, cy + h, cz - d], [cx - w, cy + h, cz - d],
        [cx - w, cy - h, cz + d], [cx + w, cy - h, cz + d],
        [cx + w, cy + h, cz + d], [cx - w, cy + h, cz + d],
    ])
    edges = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4),
             (0, 4), (1, 5), (2, 6), (3, 7)]
    xs, ys, zs = [], [], []
    for a, b in edges:
        xs += [corners[a, 0], corners[b, 0], None]
        ys += [corners[a, 1], corners[b, 1], None]
        zs += [corners[a, 2], corners[b, 2], None]
    return go.Scatter3d(x=xs, y=ys, z=zs, mode="lines",
                        line=dict(color="red", width=4), name="olcu kutusu")
