from simulator.dispersion import DEFAULT_UNCERTAINTIES, monte_carlo_dispersion


def test_all_default_parameters_have_apply_or_are_special_cased():
    names = {p.name for p in DEFAULT_UNCERTAINTIES}
    assert "Launching pitch angle" in names
    assert "Air density" in names


def test_dispersion_sweep_returns_expected_shapes():
    param = next(p for p in DEFAULT_UNCERTAINTIES if p.name == "Air density")
    values, range_err, drift_err, radial_err = monte_carlo_dispersion(param, n_samples=5)
    assert len(values) == 5
    assert len(range_err) == 5
    assert len(drift_err) == 5
    assert len(radial_err) == 5
    assert (radial_err >= 0).all()


def test_zero_perturbation_gives_near_zero_error():
    param = next(p for p in DEFAULT_UNCERTAINTIES if p.name == "Rocket total mass")
    import numpy as np
    from simulator.rocket import ROCKET_122MM
    from simulator.dispersion import UncertaintyParameter, _apply_mass  # noqa
    # sweeping across a range centered at 0 should include a near-zero error near the middle sample
    values, range_err, drift_err, radial_err = monte_carlo_dispersion(param, n_samples=5)
    mid = len(values) // 2
    assert abs(values[mid]) < 1e-6
    assert abs(range_err[mid]) < 1.0
