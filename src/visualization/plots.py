"""Reusable Plotly figure builders for the Streamlit GUI and examples.

Kept separate from src/simulator so the physics code has no plotting
dependency, and separate from src/gui so plots can be reused from plain
scripts in examples/.
"""
import numpy as np
import plotly.graph_objects as go


def trajectory_3d(north, east, altitude, title="3D Trajectory"):
    fig = go.Figure(data=[go.Scatter3d(
        x=east, y=north, z=altitude, mode="lines",
        line=dict(width=5, color=altitude, colorscale="Viridis"),
    )])
    fig.update_layout(
        title=title,
        scene=dict(xaxis_title="East [m]", yaxis_title="North [m]", zaxis_title="Altitude [m]"),
        margin=dict(l=0, r=0, t=40, b=0),
    )
    return fig


def time_series(t, y, ylabel, title, unit=""):
    fig = go.Figure(data=[go.Scatter(x=t, y=y, mode="lines")])
    fig.update_layout(title=title, xaxis_title="Flight time [s]",
                       yaxis_title=f"{ylabel}{f' [{unit}]' if unit else ''}",
                       margin=dict(l=10, r=10, t=40, b=10))
    return fig


def multi_time_series(t, series: dict, title, ylabel=""):
    """series: {label: y_array}"""
    fig = go.Figure()
    for label, y in series.items():
        fig.add_trace(go.Scatter(x=t, y=y, mode="lines", name=label))
    fig.update_layout(title=title, xaxis_title="Flight time [s]", yaxis_title=ylabel,
                       margin=dict(l=10, r=10, t=40, b=10), legend=dict(orientation="h"))
    return fig


def dispersion_scatter(values, range_err, drift_err, radial_err, param_name, unit):
    fig = go.Figure()
    fig.add_trace(go.Scatter(x=values, y=range_err, mode="lines+markers", name="Range error [m]"))
    fig.add_trace(go.Scatter(x=values, y=drift_err, mode="lines+markers", name="Drift error [m]"))
    fig.add_trace(go.Scatter(x=values, y=radial_err, mode="lines+markers", name="Radial error [m]"))
    fig.update_layout(title=f"Effect of {param_name} error on dispersion",
                       xaxis_title=f"{param_name} error [{unit}]", yaxis_title="Errors [m]",
                       margin=dict(l=10, r=10, t=40, b=10), legend=dict(orientation="h"))
    return fig


def dispersion_cloud(ranges, drifts, title="Impact point dispersion cloud"):
    fig = go.Figure(data=[go.Scatter(x=drifts, y=ranges, mode="markers",
                                       marker=dict(size=6, opacity=0.6))])
    fig.update_layout(title=title, xaxis_title="Drift [m]", yaxis_title="Range [m]",
                       margin=dict(l=10, r=10, t=40, b=10))
    return fig


def integrator_comparison(results: dict, ylabel="Altitude [m]", title="Integrator comparison"):
    """results: {label: (t, y)}"""
    fig = go.Figure()
    for label, (t, y) in results.items():
        fig.add_trace(go.Scatter(x=t, y=y, mode="lines+markers", name=label, marker=dict(size=3)))
    fig.update_layout(title=title, xaxis_title="Flight time [s]", yaxis_title=ylabel,
                       margin=dict(l=10, r=10, t=40, b=10), legend=dict(orientation="h"))
    return fig


def aero_coefficient_plot(mach, values, name, title=None):
    fig = go.Figure(data=[go.Scatter(x=mach, y=values, mode="lines+markers")])
    fig.update_layout(title=title or f"{name} vs. Mach number", xaxis_title="Mach number",
                       yaxis_title=name, margin=dict(l=10, r=10, t=40, b=10))
    return fig


def atmosphere_profile(altitudes, density, sonic_speed):
    fig = go.Figure()
    fig.add_trace(go.Scatter(x=density, y=altitudes, mode="lines", name="Density [kg/m^3]"))
    fig.update_layout(title="Air density vs. altitude", xaxis_title="Density [kg/m^3]",
                       yaxis_title="Altitude [m]", margin=dict(l=10, r=10, t=40, b=10))
    return fig
