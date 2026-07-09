from simulator.atmosphere import Atmosphere


def test_sea_level_values():
    atmo = Atmosphere()
    assert abs(atmo.temperature(0.0) - 288.15) < 1e-6
    assert abs(atmo.pressure(0.0) - 101325.0) < 1e-3
    assert 1.22 < atmo.density(0.0) < 1.23


def test_density_decreases_with_altitude():
    atmo = Atmosphere()
    rho0 = atmo.density(0.0)
    rho5000 = atmo.density(5000.0)
    rho10000 = atmo.density(10000.0)
    assert rho0 > rho5000 > rho10000


def test_sonic_speed_decreases_with_altitude_in_troposphere():
    atmo = Atmosphere()
    assert atmo.sonic_speed(0.0) > atmo.sonic_speed(10000.0)


def test_mach_number():
    atmo = Atmosphere()
    a0 = atmo.sonic_speed(0.0)
    assert abs(atmo.mach(0.0, a0) - 1.0) < 1e-6
