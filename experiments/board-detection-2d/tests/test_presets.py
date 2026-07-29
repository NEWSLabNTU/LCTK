def test_production_config_operating_point():
    from boarddet.presets import production_config
    cfg = production_config()
    assert cfg.square_icp is True
    assert cfg.isolation is True
    assert cfg.stance_floor == 0.9
    assert cfg.flatness_rms_max == 0.045
    assert cfg.up_axis == (0.0, 0.0, 1.0)


def test_production_config_per_rig_overrides():
    from boarddet.presets import production_config
    cfg = production_config(side_m=1.2, up_axis=(0.0, 1.0, 0.0),
                            cluster_min_points=20)
    assert cfg.side_m == 1.2
    assert cfg.up_axis == (0.0, 1.0, 0.0)
    assert cfg.cluster_min_points == 20
    # Defaults must be untouched by the preset (regression guard for the
    # "never change BoardConfig defaults" constraint).
    from boarddet.board_config import BoardConfig
    assert BoardConfig().square_icp is False
    assert BoardConfig().isolation is False
