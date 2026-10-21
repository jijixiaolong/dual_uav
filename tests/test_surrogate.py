import sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from event_surrogate import run, phi, lifted_response, fit_lifted, choose_c, sample_feature, TIME

def test_modes_finite():
    for mode in ['dock','undock']:
        rng=np.random.default_rng(0)
        f=sample_feature(rng,mode)
        t,theta,omega,u=run(mode,f)
        assert len(t)==len(theta)==len(omega)==len(u)
        assert np.isfinite(theta).all()
        assert np.isfinite(omega).all()

def test_learning_finite_and_bounded():
    for mode in ['dock','undock']:
        A,G=fit_lifted(mode,25,3)
        f=sample_feature(np.random.default_rng(123),mode)
        c=choose_c(mode,f,A,G)
        assert np.max(np.abs(c)) <= 6+1e-9
        assert np.isfinite(lifted_response(mode,f,c)).all()

def test_invalid_mode():
    import pytest
    with pytest.raises(ValueError):
        run('wrong',np.zeros(3))
