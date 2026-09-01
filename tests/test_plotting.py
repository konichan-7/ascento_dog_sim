from pathlib import Path

import numpy as np

from ascento_dog.plotting import save_attitude_response


def test_attitude_response_exports_csv_pdf_and_png(tmp_path: Path) -> None:
    time = np.linspace(0.0, 1.0, 101)
    active = (time >= 0.4) & (time <= 0.5)
    paths = save_attitude_response(
        time,
        np.deg2rad(5.0 * np.sin(2.0 * np.pi * time)),
        np.deg2rad(3.0 * np.sin(2.0 * np.pi * time + 0.2)),
        np.deg2rad(time),
        active,
        output_prefix=tmp_path / "attitude",
        show=False,
    )
    for path in paths:
        assert path.is_file()
        assert path.stat().st_size > 100
    assert paths[0].read_text().splitlines()[0] == (
        "time_s,roll_deg,pitch_deg,yaw_deg,disturbance_active"
    )
